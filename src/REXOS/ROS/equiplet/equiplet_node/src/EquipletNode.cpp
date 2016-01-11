/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
 *
 * @author Dennis Koole
 * @author Alexander Streng
 * @author Lars Veenendaal
 *
 * @section LICENSE
 * License: newBSD
 *
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

#include <unistd.h>
#include "equiplet_node/EquipletNode.h"
#include "equiplet_node/StateBlackboard.h"
#include "rexos_utilities/Utilities.h"
#include <rexos_configuration/Configuration.h>
#include <rexos_datatypes/OriginPlacement.h>
#include <rexos_knowledge_database/Equiplet.h>
#include <node_spawner_node/spawnNode.h>
#include <model_spawner_node/spawnPart.h>
#include <model_spawner_node/removePart.h>

#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

using namespace equiplet_node;

EquipletNode::EquipletNode(std::string equipletName, bool isSimulated, bool isShadow, std::string blackboardIp, int scadaPort) :
		EquipletStateMachine(equipletName, isSimulated),
		equipletName(equipletName),
		isSimulated(isSimulated),
		isShadow(isShadow),
		nh(),
		scada(this, &moduleRegistry, scadaPort)
{
	std::string usedImplementation = rexos_configuration::Configuration::getProperty(ROS_INTERFACE_IMPLEMENTATION_PATH, equipletName).asString();
	if(usedImplementation == BLACKBOARD_IMPLEMENTATION) {
		halInterface = new BlackBoardHalInterface(equipletName, isShadow, this, blackboardIp);
	} else if(usedImplementation == BRIDGE_IMPLEMENTATION || usedImplementation == JAVA_NODE_IMPLEMENTATION) {
		halInterface = new NodeHalInterface(equipletName, isShadow, this);
	} else {
		throw std::runtime_error("rosInterface implementation is unknown");
	}
	
	if(isShadow) {
		spawnPartClient = nodeHandle.serviceClient<model_spawner_node::spawnPart>(equipletName + "/model/spawnPart/");
		removePartClient = nodeHandle.serviceClient<model_spawner_node::removePart>(equipletName + "/model/removePart/");
		
		accelerationViolationSubscriber = 	nodeHandle.subscribe(equipletName + "/acceleration/violation/", 5, &EquipletNode::onAccelerationViolation, this);
		collisionViolationSubscriber = 		nodeHandle.subscribe(equipletName + "/collision/violation/", 5, &EquipletNode::onCollisionViolation, this);
		jointViolationSubscriber = 			nodeHandle.subscribe(equipletName + "/joint/violation/", 5, &EquipletNode::onJointViolation, this);
		accelerationViolationSubscriber = 	nodeHandle.subscribe("/acceleration/violation/", 5, &EquipletNode::onAccelerationViolation, this);
		collisionViolationSubscriber = 		nodeHandle.subscribe("/collision/violation/", 5, &EquipletNode::onCollisionViolation, this);
		jointViolationSubscriber = 			nodeHandle.subscribe("/joint/violation/", 5, &EquipletNode::onJointViolation, this);
	}
	
	REXOS_INFO_STREAM("Equiplet node started. equipletName: " << equipletName);
}

EquipletNode::~EquipletNode() {
	delete halInterface;
}

void EquipletNode::onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, rexos_datatypes::HardwareStep hardwareStep) {
	//moduleProxy->changeState(rexos_statemachine::STATE_STANDBY);
	halInterface->postHardwareStepStatus(hardwareStep);
}

void EquipletNode::onStateChanged(rexos_statemachine::State state){
	EquipletStateMachine::onStateChanged(state);
	halInterface->postStateChange(state);
}


void EquipletNode::onModeChanged(rexos_statemachine::Mode mode){
	EquipletStateMachine::onModeChanged(mode);
	halInterface->postModeChange(mode);
}
void EquipletNode::onModuleModeChanged(rexos_module::ModuleProxy* moduleProxy, 
		rexos_statemachine::Mode newMode, rexos_statemachine::Mode prevMode) {
	EquipletStateMachine::onModuleModeChanged(moduleProxy, newMode, prevMode);
	halInterface->postModeChange(moduleProxy->getModuleIdentifier(), newMode);
}
void EquipletNode::onModuleStateChanged(rexos_module::ModuleProxy* moduleProxy, 
	rexos_statemachine::State newState, rexos_statemachine::State prevState) {
	EquipletStateMachine::onModuleStateChanged(moduleProxy, newState, prevState);
	halInterface->postStateChange(moduleProxy->getModuleIdentifier(), newState);
}
std::string EquipletNode::getEquipletName() {
	return equipletName;
}
ros::NodeHandle& EquipletNode::getNodeHandle() {
	return nh;
}
/**
 * Call the lookuphandler with the data from the blackboard to get data
 *
 * @param lookupType the type of the lookup
 * @param lookupID the ID of the lookup
 * @param payload the payload, contains data that will get combined with environmentcache data
 **/
Json::Value EquipletNode::callLookupHandler(Json::Value originPlacementParameters){
	REXOS_INFO_STREAM("Called lookup handler");
 	environment_cache::getData msg;

	msg.request.identifier = originPlacementParameters["identifier"].asString();
	msg.request.paths.push_back("location/x");
	msg.request.paths.push_back("location/y");
	msg.request.paths.push_back("location/z");
	msg.request.paths.push_back("rotation/x");
	msg.request.paths.push_back("rotation/y");
	msg.request.paths.push_back("rotation/z");
	
	ros::ServiceClient lookupClient = nh.serviceClient<environment_cache::getData>(equipletName + "/getData");
	if(lookupClient.call(msg)){
		Json::Reader reader;
		Json::Value result;
		reader.parse(msg.response.jsonData, result);
		return result;
	} else {
		REXOS_WARN("Could not find anything in the lookup handler");
		return Json::Value::null;
	}
}

void EquipletNode::onHardwareStep(rexos_datatypes::HardwareStep hardwareStep) {
	REXOS_INFO_STREAM("Hardware step received");
	rexos_statemachine::Mode currentMode = getCurrentMode();
	if (currentMode != rexos_statemachine::MODE_NORMAL) {
		REXOS_WARN("Hardware step received but but cannot be processed because current mode is %s", rexos_statemachine::mode_txt[currentMode]);
		hardwareStep.setStatus(rexos_datatypes::HardwareStep::FAILED);
		halInterface->postHardwareStepStatus(hardwareStep);
		return;
	}
	rexos_statemachine::State currentState = getCurrentState();
	if (currentState != rexos_statemachine::STATE_NORMAL && currentState != rexos_statemachine::STATE_STANDBY) {
		REXOS_WARN("Hardware step received but but cannot be processed because current state is %s", rexos_statemachine::state_txt[currentState]);
		hardwareStep.setStatus(rexos_datatypes::HardwareStep::FAILED);
		halInterface->postHardwareStepStatus(hardwareStep);
		return;
	}
	
	// check originPlacement and gather required information
	rexos_datatypes::OriginPlacement originPlacement = hardwareStep.getOriginPlacement();
	if(originPlacement.getOriginPlacementType() == rexos_datatypes::OriginPlacement::RELATIVE_TO_PART_ORIGIN) {
		REXOS_DEBUG("Gathering information from the environment cache");
		Json::Value result = callLookupHandler(originPlacement.getParameters());
		if(result == Json::Value::null) {
			// could not find anything in the lookup handler, failing hardware step
			hardwareStep.setStatus(rexos_datatypes::HardwareStep::FAILED);
			halInterface->postHardwareStepStatus(hardwareStep);
			return;
		} else {
			originPlacement.setLookupResult(result);
		}
	}
	hardwareStep.setOriginPlacement(originPlacement);

	//we might still need to update the payload on the bb
	rexos_module::ModuleProxy *prox = moduleRegistry.getModule(hardwareStep.getModuleIdentifier());
	if(prox == NULL) {
		REXOS_WARN("Recieved equiplet step for module which is not in the moduleRegister");
		hardwareStep.setStatus(rexos_datatypes::HardwareStep::FAILED);
		halInterface->postHardwareStepStatus(hardwareStep);
		return;
	}
	//prox->changeState(rexos_statemachine::STATE_NORMAL);
	hardwareStep.setStatus(rexos_datatypes::HardwareStep::IN_PROGRESS);
	halInterface->postHardwareStepStatus(hardwareStep);
	prox->executeHardwareStep(hardwareStep);
}
void EquipletNode::onEquipletCommand(rexos_datatypes::EquipletCommand equipletCommand) {
	REXOS_INFO_STREAM("Equiplet command received" << equipletCommand.getCommand());
	equipletCommand.setStatus(rexos_datatypes::EquipletCommand::IN_PROGRESS);
	
	Json::Value parameters = equipletCommand.getParameters();
	if(equipletCommand.getCommand() == "changeState") {
		for(int i = 0; i < STATE_COUNT; i++) {
			if(parameters["desiredState"] == rexos_statemachine::state_txt[i]) {
				changeState(static_cast<rexos_statemachine::State>(i));
				return;
			}
		}
		REXOS_ERROR("Unknown state");
		equipletCommand.setStatus(rexos_datatypes::EquipletCommand::FAILED);
		halInterface->postEquipletCommandStatus(equipletCommand);
	} else if(equipletCommand.getCommand() == "changeMode") {
		for(int i = 0; i < MODE_COUNT; i++) {
			if(parameters["desiredMode"] == rexos_statemachine::mode_txt[i]) {
				changeMode(static_cast<rexos_statemachine::Mode>(i));
				return;
			}
		}
		REXOS_ERROR("Unknown mode");
		equipletCommand.setStatus(rexos_datatypes::EquipletCommand::FAILED);
		halInterface->postEquipletCommandStatus(equipletCommand);
	} else if(equipletCommand.getCommand() == "reload") {
		moduleRegistry.reloadModules();
	} else if(equipletCommand.getCommand() == "spawnPartModel") {
		if(isSimulated == false) {
			REXOS_WARN("Unable to spawn part because not simulated");
			return;
		}
		model_spawner_node::spawnPart call;
		call.request.partName = parameters["partName"].asString();
		call.request.originPlacementType = parameters["originPlacementType"].asString();
		call.request.positionX = parameters["positionX"].asDouble();
		call.request.positionY = parameters["positionY"].asDouble();
		call.request.positionZ = parameters["positionZ"].asDouble();
		call.request.rotationX = parameters["rotationX"].asDouble();
		call.request.rotationY = parameters["rotationY"].asDouble();
		call.request.rotationZ = parameters["rotationZ"].asDouble();
		call.request.relativeTo = parameters["relativeTo"].asString();
		call.request.spawnChildParts = parameters["spawnChildParts"].asBool();
		
		spawnPartClient.waitForExistence();
		spawnPartClient.call(call);
		
		equipletCommand.setStatus(rexos_datatypes::EquipletCommand::DONE);
		halInterface->postEquipletCommandStatus(equipletCommand);
	} else if(equipletCommand.getCommand() == "removePartModel") {
		if(isSimulated == false) {
			REXOS_WARN("Unable to spawn part because not simulated");
			return;
		}
		model_spawner_node::removePart call;
		call.request.partName = parameters["partName"].asString();
		removePartClient.waitForExistence();
		removePartClient.call(call);
		
		equipletCommand.setStatus(rexos_datatypes::EquipletCommand::DONE);
		halInterface->postEquipletCommandStatus(equipletCommand);
	} else if(equipletCommand.getCommand() == "removePartModel") {
		if(isSimulated == false) {
			REXOS_WARN("Unable to remove part because not simulated");
			return;
		}
		model_spawner_node::removePart call;
		call.request.partName = parameters["partName"].asString();
		
		removePartClient.waitForExistence();
		removePartClient.call(call);
		
		equipletCommand.setStatus(rexos_datatypes::EquipletCommand::DONE);
		halInterface->postEquipletCommandStatus(equipletCommand);
	} else {
		REXOS_WARN_STREAM("Unknown equiplet command, ignoring input");
	}
}
void EquipletNode::onAccelerationViolation(std_msgs::String message) {
	halInterface->postViolation("ACCELERATION", message.data);
}
void EquipletNode::onCollisionViolation(std_msgs::String message) {
	halInterface->postViolation("COLLISION", message.data);
}
void EquipletNode::onJointViolation(std_msgs::String message) {
	halInterface->postViolation("JOINT", message.data);
}
