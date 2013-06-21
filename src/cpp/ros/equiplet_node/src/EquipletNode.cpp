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
#include "equiplet_node/StateBlackboard.h"

#include <libjson/libjson.h>

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
		changeStateActionClient(nh, nameFromId(id) + "/change_state"),
		changeModeActionClient(nh, nameFromId(id) + "/change_mode"),
		equipletId(id),
		equipletStepBlackboardClient(NULL),
		equipletCommandBlackboardClient(NULL) 
	{

	if (mostDatabaseclient.getAllModuleData().size() > 0) {
		ROS_WARN("Previous equiplet instance did not cleanup correctly");
	}
	mostDatabaseclient.clearModuleData();
	mostDatabaseclient.setSafetyState(rexos_statemachine::STATE_SAFE);

	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "test", "equipletStepBB");
	equipletStepSubscription = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
	equipletStepBlackboardClient->subscribe(*equipletStepSubscription);

	equipletCommandBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, STATE_BLACKBOARD, COLLECTION_EQUIPLET_COMMANDS);
	equipletCommandSubscription = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
	equipletCommandBlackboardClient->subscribe(*equipletCommandSubscription);

	equipletStateBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, STATE_BLACKBOARD, COLLECTION_EQUIPLET_STATE);

	moduleRegistry.setNewRegistrationsAllowed(true);
	moduleRegistry.setModuleRegistryListener(this);

	setListener(this);

	std::cout << "Connected!" << std::endl;

	changeState(rexos_statemachine::STATE_STANDBY);
}

/**
 * Destructor for the EquipletNode
 **/
EquipletNode::~EquipletNode(){
	delete equipletStepBlackboardClient;
	delete equipletStepSubscription;
	equipletCommandBlackboardClient->unsubscribe(*equipletCommandSubscription);
	delete equipletCommandBlackboardClient;
	delete equipletCommandSubscription;
}

void EquipletNode::changeState(rexos_statemachine::State desiredState){
	ROS_INFO("changeState called with desiredState %s",rexos_statemachine::state_txt[desiredState]);
	rexos_statemachine::ChangeStateGoal goal;
	goal.desiredState = desiredState;
	changeStateActionClient.sendGoal(goal);
}

void EquipletNode::changeMode(rexos_statemachine::Mode desiredMode){
	rexos_statemachine::ChangeModeGoal goal;
	goal.desiredMode = desiredMode;
	changeModeActionClient.sendGoal(goal);
}

/**
 * This function is called when a new message on the Blackboard is received,
 * The command, destination and payload are read from the message, and the 
 * service specified in the message is called
 *
 * @param json The message parsed in the json format
 **/
void EquipletNode::onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry) {
	if(&subscription == equipletStepSubscription)
	{

	        //lets parse a root node from the bb msg.
	JSONNode n = libjson::parse(oplogEntry.getUpdateDocument().jsonString());
	std::cout << n.write() << std::endl;
	    rexos_datatypes::EquipletStep * step = new rexos_datatypes::EquipletStep(n);
	    
	    std::cout << "Step completed." << std::endl;
	    
	    std::cout << "id " << step->getId() << std::endl;
	    std::cout << "moduleId " << step->getModuleId() << std::endl;
	    std::cout << "nextStep " << step->getNextStep() << std::endl;
	    std::cout << "serviceStep Id " << step->getServiceStepID() << std::endl;
	    std::cout << "status " << step->getStatus() << std::endl;
	    
	    std::cout << "InstructData: command " << step->getInstructionData().getCommand() << std::endl;
	    std::cout << "InstructData: dest " << step->getInstructionData().getDestination() << std::endl;
	    std::cout << "InstructData: lookup " << step->getInstructionData().getLook_up() << std::endl;
	    
	    std::cout << "InstructData: LookupParams " << step->getInstructionData().getDestination() << std::endl;
	    for( map<string, string>::iterator ii=step->getInstructionData().getLook_up_parameters().begin(); ii!=step->getInstructionData().getLook_up_parameters().end(); ++ii)
	    {
	        cout << (*ii).first << ": " << (*ii).second << endl;
	    }
	    
	    std::cout << "InstructData: payload " << step->getInstructionData().getDestination() << std::endl;
	    for( map<string, string>::iterator ii=step->getInstructionData().getPayload().begin(); ii!=step->getInstructionData().getPayload().end(); ++ii)
	    {
	        cout << (*ii).first << ": " << (*ii).second << endl;
	    }
	/*std::cout << "processMessage" << std::endl;
	JSONNode message = n["message"];
	//JSONNode::const_iterator messageIt;
	std::string destination = message["destination"].as_string();
	//std::cout << "Destination " << destination << std::endl;

	std::string command = message["command"].as_string();
	//std::cout << "Command " << command << std::endl;

	std::string payload = message["payload"].write();
	std::cout << "Payload " << payload << std::endl;

	// Create the string for the service to call
	std::stringstream ss;
	ss << destination;
	ss << "/";
	ss << command;
	std::cout << ss.str() << std::endl; */
	}
	else if(&subscription == equipletCommandSubscription)
	{
		JSONNode n = libjson::parse(oplogEntry.getUpdateDocument().jsonString());
		JSONNode::const_iterator i = n.begin();

        while (i != n.end()){
            const char * node_name = i -> name().c_str();
            if (strcmp(node_name, "set_desired_state") == 0){
                changeState((rexos_statemachine::State) atoi(i -> as_string().c_str()));
            }else if (strcmp(node_name, "set_desired_mode") == 0){
                changeMode((rexos_statemachine::Mode) atoi(i -> as_string().c_str()));
            }
            i++;
        }
	}
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
void EquipletNode::callLookupHandler(std::string lookupType, std::string lookupID, environment_communication_msgs::Map payload){
 	lookup_handler::LookupServer msg;
	msg.request.lookupMsg.lookupType = lookupType;
	msg.request.lookupMsg.lookupID = lookupID;
	msg.request.lookupMsg.payLoad = payload;

	ros::NodeHandle nodeHandle;
	ros::ServiceClient lookupClient = nodeHandle.serviceClient<lookup_handler::LookupServer>("LookupHandler/lookup");
	if(lookupClient.call(msg)){
		// TODO
		// Read message
	} else {
		ROS_ERROR("Error in calling lookupHandler/lookup service");
	}
}

void EquipletNode::updateEquipletStateOnBlackboard(){
	JSONNode jsonUpdateQuery;
	jsonUpdateQuery.push_back(JSONNode("id",equipletId));

//	JSONNode body;
//	body.push_back(JSONNode("state",getCurrentState()));
//	body.push_back(JSONNode("mode",getCurrentMode()));
//	JSONNode jsonSet;
//	//jsonSet.push_back(JSONNode("$set",body.write()));

	std::ostringstream stringStream;
	stringStream << "{$set: { state: " << getCurrentState() << ",mode: " << getCurrentMode() << "}}";

	equipletStateBlackboardClient->updateDocuments(jsonUpdateQuery.write().c_str(),stringStream.str());
}

void EquipletNode::onStateChanged(){
	updateEquipletStateOnBlackboard();
}
	
void EquipletNode::onModeChanged(){
	updateEquipletStateOnBlackboard();
	
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
		std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
		for (int i = 0; i < modules.size(); i++) {
			modules[i]->changeMode(currentMode);
		}
	}
}

void EquipletNode::transitionSetup(rexos_statemachine::TransitionActionServer* as) {
	setupTransitionActionServer = as;

	moduleRegistry.setNewRegistrationsAllowed(false);

	std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(rexos_statemachine::STATE_STANDBY);
	}
	setTransitionDone(rexos_statemachine::STATE_SETUP);
}

void EquipletNode::transitionShutdown(rexos_statemachine::TransitionActionServer* as) {
	shutdownTransitionActionServer = as;

	moduleRegistry.setNewRegistrationsAllowed(true);

	std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(rexos_statemachine::STATE_SAFE);
	}
	setTransitionDone(rexos_statemachine::STATE_SHUTDOWN);
}

void EquipletNode::transitionStart(rexos_statemachine::TransitionActionServer* as) {
	as->setSucceeded();
}

void EquipletNode::transitionStop(rexos_statemachine::TransitionActionServer* as) {
	as->setSucceeded();
}

bool EquipletNode::setTransitionDone(rexos_statemachine::State transitionState){
	bool allModulesDone = false;

	std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();

	if(transitionState == rexos_statemachine::STATE_SETUP){
		bool allModulesDone = true;
		for (int i = 0; i < modules.size(); i++) {
			if(modules[i]->getCurrentState() != rexos_statemachine::STATE_STANDBY)
				allModulesDone = false;
		}
		if(allModulesDone)
			setupTransitionActionServer->setSucceeded();
	}else if(transitionState == rexos_statemachine::STATE_SHUTDOWN){
		allModulesDone = true;
		for (int i = 0; i < modules.size(); i++) {
			if(modules[i]->getCurrentState() != rexos_statemachine::STATE_SAFE)
				allModulesDone = false;
		}
		if(allModulesDone)
			shutdownTransitionActionServer->setSucceeded();
	}

	
	
	return allModulesDone;
}

void EquipletNode::onModuleStateChanged(
	ModuleProxy* moduleProxy,
	rexos_statemachine::State newState, 
	rexos_statemachine::State previousState)
{
	//ROS_INFO("EquipletNode received from %s a state change from %d to %d",moduleProxy->getModuleNodeName(),previousState,newState);
	if(getCurrentState() == rexos_statemachine::STATE_SAFE)
		setTransitionDone(rexos_statemachine::STATE_SHUTDOWN);
	else if(getCurrentState() == rexos_statemachine::STATE_STANDBY)
		setTransitionDone(rexos_statemachine::STATE_SETUP);
}

void EquipletNode::onModuleModeChanged(
	ModuleProxy* moduleProxy, 
	rexos_statemachine::Mode newMode, 
	rexos_statemachine::Mode previousMode)
{
	//ROS_INFO("ModuleRegistry received from %s a mode change from %d to %d",moduleProxy->getModuleNodeName(),previousMode,newMode);
}
