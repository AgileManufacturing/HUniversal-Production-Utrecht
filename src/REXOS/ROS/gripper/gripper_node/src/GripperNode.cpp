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
	REXOS_INFO("GripperNode Constructor entering...");
	REXOS_INFO("1");
	// get the properties and combine them for the deltarobot
	std::string properties = this->getModuleProperties();
	std::string typeProperties = this->getModuleTypeProperties();

	Json::Reader reader;
	Json::Value jsonNode;
	Json::Value typeJsonNode;
	reader.parse(properties, jsonNode);
	reader.parse(typeProperties, typeJsonNode);
	
	std::vector<std::string> typeJsonNodeMemberNames = typeJsonNode.getMemberNames();
	for(int i = 0; i < typeJsonNodeMemberNames.size(); i++) {
		jsonNode[typeJsonNodeMemberNames[i]] = typeJsonNode[typeJsonNodeMemberNames[i]];
	}
	
	gripper = new rexos_gripper::Gripper(jsonNode, this, NULL);
	setInstructionActionServer.start();
}

GripperNode::~GripperNode() {
	REXOS_INFO_STREAM("~GripperNode" << std::endl);
	delete gripper;
	// Destructor of modbus will close the modbus connection!
	delete modbus;
}

void GripperNode::onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal){
	REXOS_INFO_STREAM("handling hardwareStep: " << goal->json);
	Json::Reader reader;
	Json::Value instructionDataNode;
	reader.parse(goal->json, instructionDataNode);
	
	rexos_statemachine::SetInstructionResult result;
	result.OID = goal->OID;
	
	//if(strcmp(nodeName, "command") == 0) {
	
		if (instructionDataNode.isMember("activate") == true){
			gripper->enable();	
			setInstructionActionServer.setSucceeded(result);	
			return;
		}
		
		else if (instructionDataNode.isMember("deactivate") == true){
			gripper->disable();
			setInstructionActionServer.setSucceeded(result);	
			return;
			
		}
	
	REXOS_ERROR_STREAM("Failed setting gripper" << std::endl);
	setInstructionActionServer.setAborted(result);
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
	REXOS_INFO("Initialize transition called");
	return true;
}

bool GripperNode::transitionDeinitialize() {
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionSetup() {
	REXOS_INFO("Setup transition called");
	gripper->registerObserver(this);
	//Set currentState to start
	gripper->startWatchdog();
	
	//The service servers should be set, to provide the normal methods for the equiplet
	return true;
}

/**
 * Transition from Standby to Safe state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionShutdown() {
	REXOS_INFO("Shutdown transition called");
	// Set currentState to stop
	gripper->stopWatchdog();
	gripper->disable();

	return true;
}

/**
 * Transition from Standby to Normal state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionStart() {
	REXOS_INFO("Start transition called");

	return true;
}
/**
 * Transition from Normal to Standby state
 * @return 0 if everything went OK else error
 **/
bool GripperNode::transitionStop() {
	REXOS_INFO("Stop transition called");

	return true;
}

	void GripperNode::notifyWarning(){
		
	}
	
	void GripperNode::notifyOverheated(){
		
	}
	
	void GripperNode::notifyCooledDown(){
		
	}
	

/**
 * Main that starts the gripper node and its statemachine.
 **/
int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);
	
	if(argc < 5){
		REXOS_ERROR("Usage: gripper_node equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	std::string equipletName = argv[1];
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(argv[2], argv[3], argv[4]);
	
	REXOS_INFO("Creating GripperNode");

	GripperNode gripperNode(equipletName, moduleIdentifier);

	REXOS_INFO("Running StateEngine");
	ros::spin();
	return 0;
}
