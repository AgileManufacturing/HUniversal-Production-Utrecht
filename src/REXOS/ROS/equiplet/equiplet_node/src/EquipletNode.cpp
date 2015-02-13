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
#include "rexos_blackboard_cpp_client/FieldUpdateSubscription.h"
#include "rexos_blackboard_cpp_client/BasicOperationSubscription.h"
#include "rexos_blackboard_cpp_client/OplogEntry.h"
#include "rexos_utilities/Utilities.h"
#include <rexos_configuration/Configuration.h>
#include <rexos_knowledge_database/Equiplet.h>
#include <node_spawner_node/spawnNode.h>
#include <rexos_datatypes/OriginPlacement.h>

#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

using namespace equiplet_node;


/**
 * Create a new EquipletNode
 * @param id The unique identifier of the Equiplet
 **/
EquipletNode::EquipletNode(std::string equipletName, std::string blackboardIp, bool spawnNodesForModules) :
		nh(),
		equipletName(equipletName),
		EquipletStateMachine(equipletName),
		equipletStepBlackboardClient(NULL),
		equipletCommandBlackboardClient(NULL),
		directMoveBlackBoardClient(NULL),
		scada(this, &moduleRegistry) 
{
	if(spawnNodesForModules == true) {
		REXOS_INFO_STREAM("Spawning nodes at startup");
		rexos_knowledge_database::Equiplet equiplet = rexos_knowledge_database::Equiplet(equipletName);
		std::vector<rexos_datatypes::ModuleIdentifier> identifiers = equiplet.getModuleIdentifiersOfAttachedModules();
		
		ros::ServiceClient spanNodeClient(nh.serviceClient<node_spawner_node::spawnNode>("spawnNode"));
		for(std::vector<rexos_datatypes::ModuleIdentifier>::iterator it = identifiers.begin(); it < identifiers.end(); it++) {
			REXOS_INFO_STREAM("Spawning node for " << *it);
			node_spawner_node::spawnNode spawnNodeCall;
			spawnNodeCall.request.manufacturer = it->getManufacturer();
			spawnNodeCall.request.typeNumber = it->getTypeNumber();
			spawnNodeCall.request.serialNumber = it->getSerialNumber();
			spanNodeClient.call(spawnNodeCall);
		}
	}
	bool useCustomIp = false;
	if(blackboardIp.length() != 0) useCustomIp = true;
	
	REXOS_DEBUG("Subscribing to EquipletStepsBlackBoard");
	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(
			useCustomIp ? blackboardIp : rexos_configuration::Configuration::getProperty("rosInterface/hardwareSteps/ip", equipletName).asString(), 
			rexos_configuration::Configuration::getProperty("rosInterface/hardwareSteps/databaseName", equipletName).asString(), 
			rexos_configuration::Configuration::getProperty("rosInterface/hardwareSteps/blackboardName", equipletName).asString());
	//equipletStepSubscription = new Blackboard::FieldUpdateSubscription("status", *this);
	equipletStepSubscription = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
	//equipletStepSubscription->addOperation(Blackboard::SET);
	equipletStepBlackboardClient->subscribe(*equipletStepSubscription);
	subscriptions.push_back(equipletStepSubscription);
	sleep(1);

	REXOS_DEBUG("Subscribing to equipletCommands");
	equipletCommandBlackboardClient = new Blackboard::BlackboardCppClient(
			useCustomIp ? blackboardIp : rexos_configuration::Configuration::getProperty("rosInterface/equipletCommands/ip", equipletName).asString(), 
			rexos_configuration::Configuration::getProperty("rosInterface/equipletCommands/databaseName", equipletName).asString(), 
			rexos_configuration::Configuration::getProperty("rosInterface/equipletCommands/blackboardName", equipletName).asString());
	equipletCommandSubscription = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
	equipletCommandBlackboardClient->subscribe(*equipletCommandSubscription);
	sleep(1);
	subscriptions.push_back(equipletCommandSubscription);
	sleep(1);

	REXOS_DEBUG("Subscribing to equipletState");
	equipletStateBlackboardClient = new Blackboard::BlackboardCppClient(
			useCustomIp ? blackboardIp : rexos_configuration::Configuration::getProperty("rosInterface/equipletState/ip", equipletName).asString(), 
			rexos_configuration::Configuration::getProperty("rosInterface/equipletState/databaseName", equipletName).asString(), 
			rexos_configuration::Configuration::getProperty("rosInterface/equipletState/blackboardName", equipletName).asString());
	sleep(1);

	REXOS_INFO_STREAM("Equiplet node started. equipletName: " << equipletName);
}

/**
 * Destructor for the EquipletNode
 **/
EquipletNode::~EquipletNode(){
	for (std::vector<Blackboard::BlackboardSubscription *>::iterator iter = subscriptions.begin() ; iter != subscriptions.end() ; iter++) {
		delete *iter;
	}
	subscriptions.clear();
	
	delete equipletStepBlackboardClient;
	delete equipletStepBlackboardClient;
	delete directMoveBlackBoardClient;
	delete equipletCommandBlackboardClient;
	delete equipletStateBlackboardClient;
}

/**
 * This function is called when a new message on the Blackboard is received,
 * The command, destination and payload are read from the message, and the 
 * service specified in the message is called
 *
 * @param json The message parsed in the json format
 **/
void EquipletNode::onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry) {
	mongo::OID targetObjectId;
	oplogEntry.getTargetObjectId(targetObjectId);

	if(&subscription == equipletStepSubscription) {
		std::string jsonString = equipletStepBlackboardClient->findDocumentById(targetObjectId).jsonString();
		Json::Reader reader;
		Json::Value n;
		reader.parse(jsonString, n);
		
		rexos_datatypes::EquipletStep step = rexos_datatypes::EquipletStep(n);
	    //We only need to handle the step if its status is 'WAITING'
	    if (step.getStatus() == "WAITING") {
	    	REXOS_INFO_STREAM("handling step: " << jsonString);
    		handleHardwareStep(step, targetObjectId);
		}else if(step.getReloadEquiplet() == "RELOAD_ALL_MODULES"){
			REXOS_INFO_STREAM("HAL > ReloadEquiplet: " << jsonString);
			//reloadModules
		//	rexos_module::ModuleProxy *prox = 
		// moduleRegistry.getModule(step.getModuleIdentifier());
			moduleRegistry.reloadModules();
		//	prox->reloadModules();
			//ROS_INFO("%s ", step.getModuleIdentifier());
			equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{$set : {reloadEquiplet: \"RELOADING_COMPLETED\"} } ");
		}
	} else if(&subscription == equipletCommandSubscription) {
		ROS_INFO("Received equiplet statemachine command");
		
		Json::Reader reader;
		Json::Value n;
		reader.parse(oplogEntry.getUpdateDocument().jsonString(), n);
		
    	handleEquipletCommand(n);
	}
}

void EquipletNode::handleHardwareStep(rexos_datatypes::EquipletStep& step, mongo::OID targetObjectId){
	rexos_statemachine::Mode currentMode = getCurrentMode();
	if (currentMode != rexos_statemachine::MODE_NORMAL) {
		REXOS_WARN("Hardware step received but but cannot be processed because current mode is %s", rexos_statemachine::mode_txt[currentMode]);
		equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{$set : {status: \"FAILED\"} } ");
		return;
	}
	rexos_statemachine::State currentState = getCurrentState();
	if (currentState != rexos_statemachine::STATE_NORMAL && currentState != rexos_statemachine::STATE_STANDBY) {
		REXOS_WARN("Hardware step received but but cannot be processed because current state is %s", rexos_statemachine::state_txt[currentState]);
		equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{$set : {status: \"FAILED\"} } ");
		return;
	}
	
	// check originPlacement and gather required information
	rexos_datatypes::OriginPlacement originPlacement = step.getOriginPlacement();
	REXOS_WARN_STREAM(originPlacement.toJSON());
	if(originPlacement.getOriginPlacementType() == rexos_datatypes::OriginPlacement::RELATIVE_TO_IDENTIFIER) {
		REXOS_DEBUG("Gathering information from the environment cache");
		Json::Value result = callLookupHandler(originPlacement.getParameters());
		if(result == Json::Value::null) {
			// could not find anything in the lookup handler, failing hardware step
			equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{ $set : {status: \"FAILED\"} } ");
			return;
		} else {
			originPlacement.setLookupResult(result);
		}
	}
	step.setOriginPlacement(originPlacement);

	//we might still need to update the payload on the bb
	rexos_module::ModuleProxy *prox = moduleRegistry.getModule(step.getModuleIdentifier());
	if(prox == NULL) {
		REXOS_WARN("Recieved equiplet step for module which is not in the moduleRegister");
		equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{ $set : {status: \"FAILED\"} } ");
		return;
	}
	//prox->changeState(rexos_statemachine::STATE_NORMAL);
	equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{ $set : {status: \"IN_PROGRESS\"}  }");	
	prox->setInstruction(targetObjectId.toString(), step.toJSON());
}

void EquipletNode::handleEquipletCommand(Json::Value n) {
	// TODO is this neccesary? Does the json sometimes contain a set and sometimes not (difference between update and insert?)
	if(n.isMember("$set")) {
		REXOS_DEBUG("EquipletNode::handleEquipletCommand: Using the set entry");
		n = n["$set"];
	}
	
	if(n.isMember("desiredState")) {
		REXOS_INFO("ChangeState to %s", n["desiredState"].asCString());
		// TODO atoi?
		changeState((rexos_statemachine::State) atoi(n["desiredState"].asCString()));
	} else if(n.isMember("desiredMode")) {
		REXOS_INFO("ChangeMode to %s", n["desiredMode"].asCString());
		// TODO atoi?
		changeMode((rexos_statemachine::Mode) atoi(n["desiredState"].asCString()));
	}
}

//needed for callback ( from proxy )
void EquipletNode::onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, std::string id, bool completed) {
	//moduleProxy->changeState(rexos_statemachine::STATE_STANDBY);
	mongo::OID targetObjectId(id);

	if(completed) {
    	equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{ $set : {status: \"DONE\" } } ");
    	REXOS_INFO_STREAM("Module " << moduleInterface->getModuleIdentifier() << " is done step with id: " << id);
	} else {
    	equipletStepBlackboardClient->updateDocumentById(targetObjectId, "{ $set : {status: \"FAILED\" } } ");
    	REXOS_INFO_STREAM("Module " << moduleInterface->getModuleIdentifier() << " has failed step with id: " << id);
	}
}

void EquipletNode::onStateChanged(rexos_statemachine::State state){
	EquipletStateMachine::onStateChanged(state);
	updateEquipletStateOnBlackboard();
}

void EquipletNode::onModeChanged(rexos_statemachine::Mode mode){
	EquipletStateMachine::onModeChanged(mode);
	updateEquipletStateOnBlackboard();
}

void EquipletNode::updateEquipletStateOnBlackboard(){
	Json::Value jsonUpdateQuery;
	jsonUpdateQuery["equipletName"] = equipletName;

	std::ostringstream stringStream;
	stringStream << "{$set: { state: " << getCurrentState() << ",mode: " << getCurrentMode() << "}}";
	REXOS_INFO_STREAM("updating state on blackboard; {$set: { state: " << getCurrentState() << ",mode: " << getCurrentMode() << "}}");
	
	Json::StyledWriter writer;
	equipletStateBlackboardClient->updateDocuments(writer.write(jsonUpdateQuery), stringStream.str());
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
 	environment_cache::getData msg;

	msg.request.identifier = originPlacementParameters["identifier"].asString();
	msg.request.paths.push_back("location/x");
	msg.request.paths.push_back("location/y");
	msg.request.paths.push_back("location/z");
	msg.request.paths.push_back("rotation/x");
	msg.request.paths.push_back("rotation/y");
	msg.request.paths.push_back("rotation/z");
	
	ros::ServiceClient lookupClient = nh.serviceClient<environment_cache::getData>("getData");
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