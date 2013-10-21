/**
 * @file GripperNode.cpp
 * @brief GripperNode for controlling the gripper and accepting services.
 * @date Created: 2012-11-05
 *
 * @author Koen Braham
 * @author Dick vd Steen
 *
 * @section LICENSE
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include "gripper_node/GripperNode.h"
#include "rexos_utilities/Utilities.h"
 #include <boost/bind.hpp>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "GripperNode"
// @endcond

/**
 * The IP of the modbus we are connecting to
 **/
#define MODBUS_IP "192.168.0.2"
/**
 * The port we are connecting to
 **/
#define MODBUS_PORT 502

/**
 * Constructor
 **/
GripperNode::GripperNode(int equipletID, int moduleID) :
	rexos_statemachine::ModuleStateMachine("gripper_node", equipletID, moduleID, true),
	setInstructionActionServer(nodeHandle, "gripper_node/set_instruction", boost::bind(&GripperNode::onSetInstruction, this, _1), false) {

	std::cout << "[DEBUG] Opening modbus connection" << std::endl;
	modbusContext = modbus_new_tcp(MODBUS_IP, MODBUS_PORT);

	if (modbusContext == NULL) {
		throw std::runtime_error("Unable to allocate libmodbus context");
	}

	if (modbus_connect(modbusContext) == -1) {
		throw std::runtime_error("Modbus connection to IO controller failed");
	}

	assert(modbusContext != NULL);

	modbus = new rexos_modbus::ModbusController(modbusContext);

	std::cout << "[DEBUG] Opening IO Controller" << std::endl;
	controller = new rexos_gripper::InputOutputController(modbus);

	std::cout << "[DEBUG] Starting gripper" << std::endl;
	
	gripper = new rexos_gripper::Gripper(controller, this, wrapperForGripperError);

	// Advertise the services
	std::cout << "[DEBUG] Advertising the services" << std::endl;
	ros::NodeHandle nodeHandle;
	gripService = nodeHandle.advertiseService(GripperNodeServices::GRIP, &GripperNode::grip, this);
	releaseService = nodeHandle.advertiseService(GripperNodeServices::RELEASE, &GripperNode::release, this);
}

GripperNode::~GripperNode() {
	std::cout << "~GripperNode" << std::endl;
	delete gripper;
	// Destructor of modbus will close the modbus connection!
	delete modbus;
}

void GripperNode::onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal){
	JSONNode instructionDataNode = libjson::parse(goal->json);
	rexos_statemachine::SetInstructionResult result_;
	result_.OID = goal->OID;

    JSONNode::const_iterator i = instructionDataNode.begin();

    while (i != instructionDataNode.end()){
        const char * nodeName = i -> name().c_str();

		if(strcmp(nodeName, "command") == 0) { 
			if(strcmp(parseNodeValue("command", *i), "activate") == 0) {
				gripper->grab();
				setInstructionActionServer.setSucceeded(result_);
				return;
			} else if(strcmp(parseNodeValue("command", *i), "deactivate") == 0)
				gripper->release();
				setInstructionActionServer.setSucceeded(result_);
				return;
		}
    	++i;
	}
	std::cout << "Failed setting gripper" << std::enl;
	setInstructionActionServer.setAborted(result_);
}


/**
 * A wrapper for the gripper error handler so that we can use a member function
 * @param gripperNodeObject Pointer to the gripperTestNode object
 **/
void GripperNode::wrapperForGripperError(void* gripperNodeObject) {
	GripperNode* myself = (GripperNode*) gripperNodeObject;
	myself->error();
}

/**
 * Sends error message to equipletNode with an errorcode
 **/
void GripperNode::error() {
	//sendErrorMessage(-1);
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
void GripperNode::transitionSetup(rexos_statemachine::TransitionActionServer* as) {
	ROS_INFO("Setup transition called");
	//The service servers should be set, to provide the normal methods for the equiplet
	as->setSucceeded();
}

/**
 * Transition from Standby to Safe state
 * @return 0 if everything went OK else error
 **/
void GripperNode::transitionShutdown(rexos_statemachine::TransitionActionServer* as) {
	ROS_INFO("Shutdown transition called");
	as->setSucceeded();
}

/**
 * Transition from Standby to Normal state
 * @return 0 if everything went OK else error
 **/
void GripperNode::transitionStart(rexos_statemachine::TransitionActionServer* as) {
	ROS_INFO("Start transition called");

	// Set currentState to start
	gripper->startWatchdog();
	as->setSucceeded();
}
/**
 * Transition from Normal to Standby state
 * @return 0 if everything went OK else error
 **/
void GripperNode::transitionStop(rexos_statemachine::TransitionActionServer* as) {
	ROS_INFO("Stop transition called");

	// Set currentState to stop
	gripper->stopWatchdog();
	gripper->release();
	gripper->disable();
	as->setSucceeded();
}

/**
 * Set gripper on
 *
 * @param req The request for this service as defined in Grip.srv
 * @param res The response for this service as defined in Grip.srv
 *
 * @return true if gripper is put on else return false.
 **/
bool GripperNode::grip(gripper_node::Grip::Request &req, gripper_node::Grip::Response &res) {
	return gripper->grab();
}

/**
 * Set gripper off
 *
 * @param req The request for this service as defined in Grip.srv
 * @param res The response for this service as defined in Grip.srv
 *
 * @return true if gripper is put off else return false.
 **/
bool GripperNode::release(gripper_node::Release::Request &req, gripper_node::Release::Response &res) {
	return gripper->release();
}

std::string GripperNode::parseNodeValue(const std::string nodeName, const JSONNode & n){

	JSONNode::const_iterator i = n.begin();
	std::string result;
	while(i != n.end()){
		// get the JSON node name and value as a string
		std::string node_name = i->name();

		if(node_name == nodeName)
		{
			result = i->as_string();
		} 

		++i;
	}

	return result;
}

/**
 * Main that starts the gripper node and its statemachine.
 **/
int main(int argc, char** argv) {

	ros::init(argc, argv, NODE_NAME);
	int equipletID = 0;
	int moduleID = 0;
	
	if (argc < 3) {
		ROS_INFO("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -1;
	}

	try{
		equipletID = rexos_utilities::stringToInt(argv[1]);
		moduleID = rexos_utilities::stringToInt(argv[2]);
	} catch(std::runtime_error ex) {
		ROS_ERROR("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -2;
	}

	std::cout << "Starting gripper node" << std::endl;

	GripperNode gripperNode(equipletID, moduleID);

	ros::spin();
	return 0;
}
