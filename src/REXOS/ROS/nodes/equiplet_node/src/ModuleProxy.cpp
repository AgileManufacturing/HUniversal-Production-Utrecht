/*
 * ModuleProxy.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#include "equiplet_node/ModuleProxy.h"
#include <actionlib/client/simple_action_client.h>
#include <node_spawner_node/spawnNode.h>

namespace equiplet_node {

ModuleProxy::ModuleProxy(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier, ModuleProxyListener* mpl) :
	moduleNamespaceName(moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber()),
	equipletNamespaceName(equipletName),
	moduleIdentifier(moduleIdentifier),
	changeStateActionClient(nodeHandle, equipletNamespaceName + "/" + moduleNamespaceName + "/change_state"),
	changeModeActionClient(nodeHandle, equipletNamespaceName + "/" + moduleNamespaceName + "/change_mode"),
	setInstructionActionClient(nodeHandle, equipletNamespaceName + "/" + moduleNamespaceName + "/set_instruction"),
	currentMode(rexos_statemachine::Mode::MODE_SERVICE),
	currentState(rexos_statemachine::State::STATE_OFFLINE),
	moduleProxyListener(mpl),
	connectedWithNode(false),
	bond(NULL)
{
	stateUpdateServiceServer = nodeHandle.advertiseService(
			equipletNamespaceName + "/" + moduleNamespaceName + "/state_update",
			&ModuleProxy::onStateChangeServiceCallback, this);

	modeUpdateServiceServer = nodeHandle.advertiseService(
			equipletNamespaceName + "/" + moduleNamespaceName + "/mode_update",
			&ModuleProxy::onModeChangeServiceCallback, this);
	
	ROS_INFO_STREAM("Setting state action client: " << equipletNamespaceName + "/" + moduleNamespaceName << "/change_state");
	ROS_INFO_STREAM("Setting mode action client: " << equipletNamespaceName + "/" + moduleNamespaceName << "/change_mode");
	ROS_INFO_STREAM("Setting instruction action client: " << equipletNamespaceName + "/" + moduleNamespaceName << "/set_instruction");

}

ModuleProxy::~ModuleProxy() {
	delete bond;
	// TODO Auto-generated destructor stub
}

rexos_statemachine::State ModuleProxy::getCurrentState(){
	return currentState;
}

rexos_statemachine::Mode ModuleProxy::getCurrentMode(){
	return currentMode;
}

rexos_knowledge_database::ModuleIdentifier ModuleProxy::getModuleIdentifier(){
	return moduleIdentifier;
}

void ModuleProxy::setModuleProxyListener(ModuleProxyListener* mpl){
	moduleProxyListener = mpl;
}

void ModuleProxy::changeState(rexos_statemachine::State state) {
	ROS_INFO("ModuleProxy of %s send new state goal %s", moduleIdentifier.toString().c_str(), rexos_statemachine::state_txt[state]);
	ROS_INFO_STREAM("state " << rexos_statemachine::state_txt[state] << " " << rexos_statemachine::state_txt[getCurrentState()]);
	ROS_INFO("a");
	if(state == rexos_statemachine::State::STATE_SAFE && getCurrentState() == rexos_statemachine::State::STATE_OFFLINE) {
	ROS_INFO("b");
		if(connectedWithNode == false) {
	ROS_INFO("c");
			ros::ServiceClient spanNodeClient(nodeHandle.serviceClient<node_spawner_node::spawnNode>("spawnNode"));
			ROS_INFO_STREAM("Spawning node for " << moduleIdentifier);
			node_spawner_node::spawnNode spawnNodeCall;
			spawnNodeCall.request.manufacturer = moduleIdentifier.getManufacturer();
			spawnNodeCall.request.typeNumber = moduleIdentifier.getTypeNumber();
			spawnNodeCall.request.serialNumber = moduleIdentifier.getSerialNumber();
			spanNodeClient.call(spawnNodeCall);
			
			// wait for the node to come online
	ROS_INFO("d");
			if(connectedWithNode == false) {
	ROS_INFO("e");
				boost::unique_lock<boost::mutex> lock(nodeStartupMutex);
				nodeStartupCondition.wait(lock);
			}
		} else {
			ROS_WARN("Node has already been stated, which is not expected (did someone manually start this node?)");
		}
	}
	
	ROS_INFO("f");
	desiredState = state;
	
	rexos_statemachine::ChangeStateGoal goal;
	goal.desiredState = desiredState;
	changeStateActionClient.waitForServer();
	ROS_INFO("g");
	changeStateActionClient.sendGoal(goal, 
			actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction>::SimpleDoneCallback(), 
			actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction>::SimpleActiveCallback(), 
			boost::bind(&ModuleProxy::onModuleTransitionFeedbackCallback, this, _1));
	ROS_INFO("h");
}

void ModuleProxy::changeMode(rexos_statemachine::Mode mode) {
	ROS_INFO("ModuleProxy of %s send new mode goal %s", moduleIdentifier.toString().c_str(), rexos_statemachine::mode_txt[mode]);
	rexos_statemachine::ChangeModeGoal goal;
	goal.desiredMode = mode;
	changeModeActionClient.sendGoal(goal);
}

void ModuleProxy::setInstruction(std::string OID, JSONNode n) {
	ROS_INFO_STREAM("Sent Instruction to module: " << moduleIdentifier.toString().c_str());
	if(connectedWithNode == false) {
		ROS_ERROR("Sent intruction to module which is not connected to the ROS node");
		return;
	}
	rexos_statemachine::SetInstructionGoal goal;

	goal.json = n.write();
	goal.OID = OID;

	setInstructionActionClient.sendGoal(goal, boost::bind(&ModuleProxy::onInstructionServiceCallback, this, _1, _2), NULL, NULL);
}
void ModuleProxy::goToNextTransitionPhase() {
	rexos_statemachine::ChangeStateGoal goal;
	goal.desiredState = desiredState;
	changeStateActionClient.waitForServer();
	changeStateActionClient.sendGoal(goal, 
			actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction>::SimpleDoneCallback(), 
			actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction>::SimpleActiveCallback(), 
			boost::bind(&ModuleProxy::onModuleTransitionFeedbackCallback, this, _1));
}

bool ModuleProxy::onStateChangeServiceCallback(StateUpdateRequest &req, StateUpdateResponse &res){
	//ROS_INFO("ModuleProxy of %s received state change to %s", moduleNodeName.c_str(), rexos_statemachine::state_txt[currentState]);

	rexos_statemachine::State previousState = currentState;
	currentState = static_cast<rexos_statemachine::State>(req.state);

	if(moduleProxyListener != NULL){
		moduleProxyListener->onModuleStateChanged(this,currentState,previousState);
	}

	return true;
}

bool ModuleProxy::onModeChangeServiceCallback(ModeUpdateRequest &req, ModeUpdateResponse &res){
	//ROS_INFO("ModuleProxy of %s received mode change to %s", moduleNodeName.c_str(), rexos_statemachine::Mode_txt[currentMode]);

	rexos_statemachine::Mode previousMode = currentMode;
	currentMode = static_cast<rexos_statemachine::Mode>(req.mode);

	if(moduleProxyListener != NULL){
		moduleProxyListener->onModuleModeChanged(this,currentMode,previousMode);
	}

	return true;
}

void ModuleProxy::onInstructionServiceCallback(const actionlib::SimpleClientGoalState& state, const rexos_statemachine::SetInstructionResultConstPtr& result){

	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		moduleProxyListener->onInstructionStepCompleted(this, result->OID, true);
	else
		moduleProxyListener->onInstructionStepCompleted(this, result->OID, false);
}

void ModuleProxy::onModuleTransitionFeedbackCallback(const rexos_statemachine::ChangeStateFeedbackConstPtr& feedback) {
	ROS_INFO("Recieved a feedback call");
	std::vector<rexos_knowledge_database::SupportedMutation> supportedMutations;
	for(int i = 0; i < feedback->gainedSupportedMutations.size(); i++) {
		rexos_knowledge_database::SupportedMutation supportedMutation(
				feedback->gainedSupportedMutations.at(i));
		supportedMutations.push_back(supportedMutation);
	}
	std::vector<rexos_knowledge_database::RequiredMutation> requiredMutations;
	for(int i = 0; i < feedback->requiredMutationsRequiredForNextPhase.size(); i++) {
		rexos_knowledge_database::RequiredMutation requiredMutation(
				feedback->requiredMutationsRequiredForNextPhase.at(i).mutation, feedback->requiredMutationsRequiredForNextPhase.at(i).isOptional);
		requiredMutations.push_back(requiredMutation);
	}
	moduleProxyListener->onModuleTransitionPhaseCompleted(this, supportedMutations, requiredMutations);
}

void ModuleProxy::onBondCallback(rexos_bond::Bond* bond, Event event){
	if(event == FORMED) {
		ROS_INFO("Bond has been formed");
		connectedWithNode = true;
		nodeStartupCondition.notify_one();
	} else {
		ROS_WARN("Bond has been broken");
		moduleProxyListener->onModuleDied(this);
		connectedWithNode = false;
		delete bond;
		bond = NULL;
	}
}
void ModuleProxy::bind() {
	ROS_INFO_STREAM("binding B on " << (equipletNamespaceName + "/bond")<< " id " << moduleNamespaceName);
	bond = new rexos_bond::Bond(equipletNamespaceName + "/bond", moduleNamespaceName, this);
	bond->start();
}
} /* namespace equiplet_node */