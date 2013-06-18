/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
 *
 * @author Joris Vergeer & Gerben Boot
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

#include <equiplet_node/EquipletNode.h>
#include <rexos_statemachine/ChangeStateAction.h>
#include <rexos_statemachine/ChangeModeAction.h>

using namespace equiplet_node;

/**
 * Create a new EquipletNode
 * @param id The unique identifier of the Equiplet
 **/
EquipletNode::EquipletNode(int id, std::string blackboardIp) :
		StateMachine(nameFromId(id)),
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

	blackboardClient = new BlackboardCppClient(blackboardIp, "REXOS",
			"blackboard", this);
	blackboardClient->subscribe("instruction");

	std::cout << "Connected!" << std::endl;

	moduleRegistry.setNewRegistrationsAllowed(true);
}

/**
 * Destructor for the EquipletNode
 **/
EquipletNode::~EquipletNode() {
	delete blackboardClient;
}

/**
 * This function is called when a new message on the Blackboard is received,
 * The command, destination and payload are read from the message, and the 
 * service specified in the message is called
 *
 * @param json The message parsed in the json format
 **/
void EquipletNode::blackboardReadCallback(std::string json) {
	std::cout << "processMessage" << std::endl;
	JSONNode n = libjson::parse(json);
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
	blackboardClient->removeOldestMessage();
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
void EquipletNode::callLookupHandler(std::string lookupType,
		std::string lookupID, environment_communication_msgs::Map payload) {
	lookup_handler::LookupServer msg;
	msg.request.lookupMsg.lookupType = lookupType;
	msg.request.lookupMsg.lookupID = lookupID;
	msg.request.lookupMsg.payLoad = payload;

	ros::NodeHandle nodeHandle;
	ros::ServiceClient lookupClient = nodeHandle.serviceClient<
			lookup_handler::LookupServer>("LookupHandler/lookup");
	if (lookupClient.call(msg)) {
		// TODO
		// Read message
	} else {
		ROS_ERROR("Error in calling lookupHandler/lookup service");
	}
}

bool EquipletNode::changeModuleState(int moduleID,rexos_statemachine::State state){
//	rexos_most::ChangeStateGoal_ goal;
//	goal.desiredState state;
//	changeStateClient.sendGoal(goal,);
//
//	rexos_most::ChangeState::Request req;
//	rexos_most::ChangeState::Response res;
//	req.desiredState = rexos_most::STATE_SAFE;
//	std::stringstream ss;
//	ss << "/most/" << moduleID << "/changeState";
//	if(!(nh.serviceClient<rexos_most::ChangeState>(
//		ss.str()).call(req, res)) ||
//		!res.executed)
//	{
//		return false;
//	}
	return true;
}

void EquipletNode::transitionSetup() {
	moduleRegistry.setNewRegistrationsAllowed(false);

	std::vector<ModuleProxy*> modules = moduleRegistry.getRigisteredModules();
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(rexos_statemachine::STATE_STANDBY);
	}
}

void EquipletNode::transitionShutdown() {
	moduleRegistry.setNewRegistrationsAllowed(true);

	std::vector<ModuleProxy*> modules = moduleRegistry.getRigisteredModules();
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(rexos_statemachine::STATE_SAFE);
	}
}

void EquipletNode::transitionStart() {
}

void EquipletNode::transitionStop() {
}

//bool EquipletNode::moduleUpdateService(rexos_most::ModuleUpdate::Request& req,
//		rexos_most::ModuleUpdate::Response& res) {
//	ROS_INFO("Received module update (id=%d, state=%s, modi=%s)", req.info.id,
//			rexos_statemachine::MOSTState_txt[req.info.state],rexos_statemachine::MOSTModi_txt[req.info.modi]);
//	MOSTDatabaseClient::ModuleData data;
//	data.id = req.info.id;
//	data.state = req.info.state;
//	data.modi = req.info.modi;
//	mostDatabaseclient.setModuleData(data);
//
//	rexos_statemachine::State safetyState = rexos_statemachine::STATE_SAFE;
//	auto moduleDatas = mostDatabaseclient.getAllModuleData();
//	for (auto it = moduleDatas.begin(); it != moduleDatas.end(); it++) {
//		rexos_statemachine::State modState = (rexos_statemachine::State) it->state;
//		rexos_statemachine::State roundedState;
//		switch (modState) {
//		case rexos_statemachine::STATE_SAFE:
//			roundedState = rexos_statemachine::STATE_SAFE;
//			break;
//		case rexos_statemachine::STATE_SETUP:
//		case rexos_statemachine::STATE_SHUTDOWN:
//		case rexos_statemachine::STATE_STANDBY:
//			roundedState = rexos_statemachine::STATE_STANDBY;
//			break;
//		case rexos_statemachine::STATE_START:
//		case rexos_statemachine::STATE_STOP:
//		case rexos_statemachine::STATE_NORMAL:
//			roundedState = rexos_statemachine::STATE_NORMAL;
//		}
//
//		if(roundedState > safetyState){
//			safetyState = roundedState;
//		}
//
//		mostDatabaseclient.setSafetyState(safetyState);
//	}
//	return true;
//}
