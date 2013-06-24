/*
 * ModuleProxy.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#include "equiplet_node/ModuleProxy.h"

namespace equiplet_node {

ModuleProxy::ModuleProxy(std::string equipletNodeName, std::string moduleName, int equipletId, int moduleId, ModuleProxyListener* mpl):
	moduleNodeName(moduleName + "_" + std::to_string(equipletId) + "_" + std::to_string(moduleId)),
	changeStateActionClient(nodeHandle, moduleNodeName + "/change_state"),
	changeModeActionClient(nodeHandle, moduleNodeName + "/change_mode"),
	setInstructionActionClient(nodeHandle, moduleName + "/set_instruction"),
	currentMode(rexos_statemachine::Mode::MODE_NORMAL),
	currentState(rexos_statemachine::State::STATE_SAFE),
	moduleProxyListener(mpl),
	moduleId(moduleId)
{
	stateUpdateServiceServer = nodeHandle.advertiseService(
			equipletNodeName + "/" + moduleNodeName + "/state_update",
			&ModuleProxy::onStateChangeServiceCallback, this);

	modeUpdateServiceServer = nodeHandle.advertiseService(
			equipletNodeName + "/" + moduleNodeName + "/mode_update",
			&ModuleProxy::onModeChangeServiceCallback, this);

	instructionUpdateServiceServer = nodeHandle.advertiseService(
			equipletNodeName + "/" + moduleNodeName + "/instruction_update",
			&ModuleProxy::onInstructionServiceCallback, this);
}

ModuleProxy::~ModuleProxy() {
	// TODO Auto-generated destructor stub
}

rexos_statemachine::State ModuleProxy::getCurrentState(){
	return currentState;
}

rexos_statemachine::Mode ModuleProxy::getCurrentMode(){
	return currentMode;
}

int ModuleProxy::getModuleId(){
	return moduleId;
}

std::string ModuleProxy::getModuleNodeName(){
	return moduleNodeName;
}

void ModuleProxy::setModuleProxyListener(ModuleProxyListener* mpl){
	moduleProxyListener = mpl;
}

void ModuleProxy::changeState(rexos_statemachine::State state) {
	ROS_INFO("ModuleProxy of %s send new state goal %s", moduleNodeName.c_str(), rexos_statemachine::state_txt[state]);
	rexos_statemachine::ChangeStateGoal goal;
	goal.desiredState = state;
	changeStateActionClient.sendGoal(goal);
}

void ModuleProxy::changeMode(rexos_statemachine::Mode mode) {
	ROS_INFO("ModuleProxy of %s send new mode goal %s", moduleNodeName.c_str(), rexos_statemachine::Mode_txt[mode]);
	rexos_statemachine::ChangeModeGoal goal;
	goal.desiredMode = mode;
	changeModeActionClient.sendGoal(goal);
}

void ModuleProxy::setInstruction(JSONNode n) {
	std::cout << "JSON van module: " <<  moduleNodeName.c_str() << " verzonden" << std::endl;
	rexos_statemachine::SetInstructionGoal goal;
	goal.json = n.write();
	setInstructionActionClient.sendGoal(goal);
}

/* bool ModuleProxy::setModuleInstruction(){
	changeState(rexos_statemachine::STATE_NORMAL);
	changeStateActionClient.waitForResult();
	if (changeStateActionClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		return false;

	//set instruction data

	changeState(rexos_statemachine::STATE_STANDBY);
} */

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

bool ModuleProxy::onInstructionServiceCallback(SetInstructionRequest &req, SetInstructionResponse &res){
	ROS_INFO("ModuleProxy processed received instruction callbek");
		//Report back to the proxylistener -> moduleProxyListener->onModuleInstruction(this,currentMode,previousMode);
	return true;
}

} /* namespace equiplet_node */