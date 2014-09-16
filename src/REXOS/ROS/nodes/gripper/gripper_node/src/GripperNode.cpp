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

#include <jsoncpp/json/reader.h>


// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "GripperNode"
// @endcond

GripperNode::GripperNode(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier) :
		rexos_knowledge_database::Module(moduleIdentifier),
		rexos_statemachine::ModuleStateMachine(equipletName, moduleIdentifier, true),
		setInstructionActionServer(
				nodeHandle, 
				equipletName + "/" + moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber() + "/set_instruction", 
				boost::bind(&GripperNode::onSetInstruction, this, _1), 
				false) {
	ROS_INFO("GripperNode Constructor entering...");
	ROS_INFO("1");
	// get the properties and combine them for the deltarobot
	std::string properties = this->getModuleProperties();
	std::string typeProperties = this->getModuleTypeProperties();

	Json::Reader reader;
	Json::Value jsonNode;
	Json::Value typeJsonNode;
	reader.parse(properties, jsonNode);
	reader.parse(typeProperties, typeJsonNode);
	
	gripper = new rexos_gripper::Gripper(jsonNode, this, NULL);
	setInstructionActionServer.start();
}

GripperNode::~GripperNode() {
	std::cout << "~GripperNode" << std::endl;
	delete gripper;
	// Destructor of modbus will close the modbus connection!
	delete modbus;
}

void GripperNode::onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal){
	Json::Reader reader;
	Json::Value instructionDataNode;
	reader.parse(goal->json, instructionDataNode);
	
	ROS_WARN_STREAM(goal->json);
	rexos_statemachine::SetInstructionResult result_;
	result_.OID = goal->OID;

	/*
	
	JSONNode::const_iterator i = instructionDataNode.begin();

    while (i != instructionDataNode.end()){

        const char * nodeName = i -> name().c_str();
		ROS_WARN_STREAM("command " << nodeName);

		if(strcmp(nodeName, "command") == 0) {
			std::string value = parseNodeValue("command", instructionDataNode);

			ROS_WARN_STREAM("value " << value);
			if(strcmp(value.c_str(), "activate") == 0) {
				std::cout << "Activating gripper" << std::endl;
				//gripper->grab();
				//std::cout << "Gripper activated" << std::endl;
				setInstructionActionServer.setSucceeded(result_);
				std::cout << "setSucceeded" << std::endl;
				return;
			} else if(strcmp(value.c_str(), "deactivate") == 0) {
				std::cout << "Deactivating gripper" << std::endl;
				//gripper->release();
				//std::cout << "Gripper deactivated" << std::endl;
				setInstructionActionServer.setSucceeded(result_);
				std::cout << "setSucceeded" << std::endl;
				return;
			}
		}
    	++i;
	}*/

	std::cout << "Failed setting gripper" << std::endl;
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

bool GripperNode::transitionInitialize() {
	ROS_INFO("Initialize transition called");
	return true;
}

bool GripperNode::transitionDeinitialize() {
	ROS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionSetup() {
	ROS_INFO("Setup transition called");
	// Set currentState to start
	//gripper->startWatchdog();
	
	//The service servers should be set, to provide the normal methods for the equiplet
	return true;
}

/**
 * Transition from Standby to Safe state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionShutdown() {
	ROS_INFO("Shutdown transition called");
	// Set currentState to stop
	//gripper->stopWatchdog();
	//gripper->release();
	//gripper->disable();

	return true;
}

/**
 * Transition from Standby to Normal state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionStart() {
	ROS_INFO("Start transition called");

	return true;
}
/**
 * Transition from Normal to Standby state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionStop() {
	ROS_INFO("Stop transition called");

	return true;
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

/**
 * Main that starts the gripper node and its statemachine.
 **/
int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);
	
	if(argc < 5){
		ROS_ERROR("Usage: gripper_node equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	std::string equipletName = argv[1];
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(argv[2], argv[3], argv[4]);
	
	ROS_INFO("Creating GripperNode");

	GripperNode gripperNode(equipletName, moduleIdentifier);

	ROS_INFO("Running StateEngine");
	ros::spin();
	return 0;
}
