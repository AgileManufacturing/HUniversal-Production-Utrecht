/**
 * @file GripperNode.cpp
 * @brief GripperNode for controlling the gripper and accepting services.
 * @date Created: 2012-11-05
 *
 * @author Koen Braham
 * @author Dick vd Steen
 * @author Peter Markotic
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
#include <rexos_datatypes/EquipletStep.h>
#include <boost/bind.hpp>
#include <rexos_datatypes/EquipletStep.h>
#include <jsoncpp/json/reader.h>

using namespace gripper_node;

GripperNode::GripperNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, bool isShadow) :
		rexos_module::ActorModule(equipletName, moduleIdentifier, isSimulated, isShadow) {
	REXOS_INFO("GripperNode Constructor entering...");
	// get the properties and combine them for the deltarobot
	std::string properties = this->getModuleProperties();
	std::string typeProperties = this->getModuleTypeProperties();

	Json::Reader reader;
	Json::Value jsonNode;
	Json::Value typeJsonNode;
	reader.parse(properties, jsonNode);
	reader.parse(typeProperties, typeJsonNode);
	
	std::vector<std::string> typeJsonNodeMemberNames = typeJsonNode.getMemberNames();
	for(uint i = 0; i < typeJsonNodeMemberNames.size(); i++) {
		jsonNode[typeJsonNodeMemberNames[i]] = typeJsonNode[typeJsonNodeMemberNames[i]];
	}
	
	gripper = new rexos_gripper::Gripper(jsonNode);
}

GripperNode::~GripperNode() {
	REXOS_INFO_STREAM("~GripperNode" << std::endl);
	delete gripper;
	// Destructor of modbus will close the modbus connection!
	delete modbus;
}

void GripperNode::onSetInstruction(const rexos_module::SetInstructionGoalConstPtr &goal){
	REXOS_INFO_STREAM("handling hardwareStep: " << goal->json);
	Json::Reader reader;
	Json::Value equipletStepNode;
	reader.parse(goal->json, equipletStepNode);
	rexos_datatypes::EquipletStep equipletStep(equipletStepNode);
	
	rexos_module::SetInstructionResult result;
	result.OID = goal->OID;
	
	Json::Value instructionData = equipletStep.getInstructionData();
	if (instructionData.isMember("activate") == true){
		gripper->activate();
		ros::Duration(0.1).sleep();
		setInstructionActionServer.setSucceeded(result);	
		std::cout << "Gripper activated" << std::endl;
		return;
	} else if (instructionData.isMember("deactivate") == true){
		gripper->deactivate();
		ros::Duration(0.1).sleep();
		setInstructionActionServer.setSucceeded(result);	
		std::cout << "Gripper deactivated" << std::endl;
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
	gripper->deactivate();
	
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

//TODO: Implement the following methods, there must be a MAST functionallity implemented here when a certain event occurs. Like when the gripper is overheated,
//the gripper should look for a save place to drop the current item.
void GripperNode::notifyWarned(){
	REXOS_INFO("GripperNode is Warned ");
}
	
void GripperNode::notifyOverheated(){
	REXOS_INFO("GripperNode is Overheated");
}
	
void GripperNode::notifyCooledDown(){
	REXOS_INFO("GripperNode is CooledDown");
}
	

/**
 * Main that starts the gripper node and its statemachine.
 **/
int main(int argc, char** argv) {
	if(argc < 5){
		REXOS_ERROR("Usage: gripper_node (--isSimulated | --isShadow) equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	bool isSimulated = false;
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--isSimulated") {
			isSimulated = true;
		} else if (arg == "--isShadow") {
			isShadow = true;
			isSimulated = true;
		}
	}
	
	std::string equipletName = std::string(argv[argc - 4]);
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[argc - 3], argv[argc - 2], argv[argc - 1]);
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_" + moduleIdentifier.getManufacturer() + "_" + 
			moduleIdentifier.getTypeNumber() + "_" + moduleIdentifier.getSerialNumber();
	ros::init(argc, argv, nodeName);
	
	GripperNode gripperNode(equipletName, moduleIdentifier, isSimulated, isShadow);
	
	ros::spin();
	return 0;
}
