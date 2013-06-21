

/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
 *
 * @author Dennis Koole
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
#include "rexos_blackboard_cpp_client/BlackboardCppClient.h"
#include "rexos_blackboard_cpp_client/BasicOperationSubscription.h"
#include "rexos_blackboard_cpp_client/OplogEntry.h"
#include "rexos_utilities/Utilities.h"
#include <rexos_statemachine/ChangeStateAction.h>
#include <rexos_statemachine/ChangeModeAction.h>

using namespace equiplet_node;

/**
 * Create a new EquipletNode
 * @param id The unique identifier of the Equiplet
 **/
EquipletNode::EquipletNode(int id, std::string blackboardIp) :
		StateMachine(nameFromId(id),
			{rexos_statemachine::MODE_NORMAL, 
			rexos_statemachine::MODE_SERVICE, 
			rexos_statemachine::MODE_ERROR,
			rexos_statemachine::MODE_CRITICAL_ERROR, 
			rexos_statemachine::MODE_E_STOP,
			rexos_statemachine::MODE_LOCK,
			rexos_statemachine::MODE_STEP}
		),
		moduleRegistry(nameFromId(id), id),
		equipletId(id),
		blackboardClient(NULL) {

	if (mostDatabaseclient.getAllModuleData().size() > 0) {
		ROS_WARN("Previous equiplet instance did not cleanup correctly");
	}
	mostDatabaseclient.clearModuleData();
	mostDatabaseclient.setSafetyState(rexos_statemachine::STATE_SAFE);

//	moduleUpdateServiceServer = nh.advertiseService(
//			"/most/equiplet/moduleUpdate", &EquipletNode::moduleUpdateService,
//			this);

	blackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "test", "equipletStepBB");
	Blackboard::BasicOperationSubscription * sub = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
	subscriptions.push_back(sub);
	blackboardClient->subscribe(*sub);

	moduleRegistry.setNewRegistrationsAllowed(true);
	moduleRegistry.setModuleRegistryListener(this);

	setListener(this);

	std::cout << "Connected!" << std::endl;
}

/**
 * Destructor for the EquipletNode
 **/
EquipletNode::~EquipletNode(){
	delete blackboardClient;
	for (std::vector<Blackboard::BlackboardSubscription *>::iterator iter = subscriptions.begin() ; iter != subscriptions.end() ; iter++) {
		delete *iter;
	}
	subscriptions.clear();
}

/**
 * This function is called when a new message on the Blackboard is received,
 * The command, destination and payload are read from the message, and the 
 * service specified in the message is called
 *
 * @param json The message parsed in the json format
 **/
void EquipletNode::onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry) {
        //lets parse a root node from the bb msg.
	JSONNode n = libjson::parse(oplogEntry.getUpdateDocument().jsonString());
	std::cout << n.write() << std::endl;
    rexos_datatypes::EquipletStep * step = new rexos_datatypes::EquipletStep(n);

}

std::string EquipletNode::getName() {
	return nameFromId(equipletId);
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
void EquipletNode::callLookupHandler(std::string lookupType, std::string lookupID, std::map<std::string, std::string> payloadMap){
 	lookup_handler::LookupServer msg;
	msg.request.lookupMsg.lookupType = lookupType;
	msg.request.lookupMsg.lookupID = lookupID;
	msg.request.lookupMsg.payLoad = createMapMessage(payloadMap);

	ros::NodeHandle nodeHandle;
	ros::ServiceClient lookupClient = nodeHandle.serviceClient<lookup_handler::LookupServer>("LookupHandler/lookup");
	if(lookupClient.call(msg)){
		// TODO
		// Read message
	} else {
		ROS_ERROR("Error in calling lookupHandler/lookup service");
	}
}

/**
 * Create a Map message from a map with strings as keys and strings as values
 *
 * @param Map The map to convert
 *
 * @return environment_communication_msgs::Map The map message object
 **/
environment_communication_msgs::Map EquipletNode::createMapMessage(std::map<std::string, std::string> &Map){
	std::map<std::string, std::string>::iterator MapIterator;
	environment_communication_msgs::Map mapMsg;
	environment_communication_msgs::KeyValuePair prop;
	for(MapIterator = Map.begin(); MapIterator != Map.end(); MapIterator++){
		prop.key = (*MapIterator).first;
		prop.value = (*MapIterator).second;
		mapMsg.map.push_back(prop);
	}
	return mapMsg;
}


void EquipletNode::onStateChanged(){
}

void EquipletNode::onModeChanged(){
	bool changeModuleModes = false;

	rexos_statemachine::Mode currentMode = getCurrentMode();
	switch(currentMode){
		case rexos_statemachine::MODE_NORMAL:	
		case rexos_statemachine::MODE_ERROR:
		case rexos_statemachine::MODE_CRITICAL_ERROR:
		case rexos_statemachine::MODE_E_STOP:	
			changeModuleModes = true; break;
	}

	if(changeModuleModes){
		std::vector<ModuleProxy*> modules = moduleRegistry.getRigisteredModules();
		for (int i = 0; i < modules.size(); i++) {
			modules[i]->changeMode(currentMode);
		}
	}
}

void EquipletNode::transitionSetup(rexos_statemachine::TransitionActionServer* as) {
	setupTransitionActionServer = as;

	moduleRegistry.setNewRegistrationsAllowed(false);

	std::vector<ModuleProxy*> modules = moduleRegistry.getRigisteredModules();
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(rexos_statemachine::STATE_STANDBY);
	}
}

void EquipletNode::transitionShutdown(rexos_statemachine::TransitionActionServer* as) {
	shutdownTransitionActionServer = as;

	moduleRegistry.setNewRegistrationsAllowed(true);

	std::vector<ModuleProxy*> modules = moduleRegistry.getRigisteredModules();
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(rexos_statemachine::STATE_SAFE);
	}
}

void EquipletNode::transitionStart(rexos_statemachine::TransitionActionServer* as) {
	as->setSucceeded();
}

void EquipletNode::transitionStop(rexos_statemachine::TransitionActionServer* as) {
	as->setSucceeded();
}

void EquipletNode::onModuleStateChanged(
	ModuleProxy* moduleProxy,
	rexos_statemachine::State newState, 
	rexos_statemachine::State previousState)
{
	//ROS_INFO("EquipletNode received from %s a state change from %d to %d",moduleProxy->getModuleNodeName(),previousState,newState);

	if(getCurrentState() == rexos_statemachine::STATE_SETUP)
	{
		bool allModulesStandby = true;

		std::vector<ModuleProxy*> modules = moduleRegistry.getRigisteredModules();
		for (int i = 0; i < modules.size(); i++) {
			if(modules[i]->getCurrentState() != rexos_statemachine::STATE_STANDBY)
				allModulesStandby = false;
		}

		if(allModulesStandby)
			setupTransitionActionServer->setSucceeded();
	}else if(getCurrentState() == rexos_statemachine::STATE_SHUTDOWN)
	{
		bool allModulesSafe = true;

		std::vector<ModuleProxy*> modules = moduleRegistry.getRigisteredModules();
		for (int i = 0; i < modules.size(); i++) {
			if(modules[i]->getCurrentState() != rexos_statemachine::STATE_SAFE)
				allModulesSafe = false;
		}

		if(allModulesSafe)
			shutdownTransitionActionServer->setSucceeded();
	}
}

void EquipletNode::onModuleModeChanged(
	ModuleProxy* moduleProxy, 
	rexos_statemachine::Mode newMode, 
	rexos_statemachine::Mode previousMode)
{
	//ROS_INFO("ModuleRegistry received from %s a mode change from %d to %d",moduleProxy->getModuleNodeName(),previousMode,newMode);
}