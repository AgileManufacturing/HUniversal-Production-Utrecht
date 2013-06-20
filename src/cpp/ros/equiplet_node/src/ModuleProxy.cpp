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
	ROS_INFO("ModuleProxy of %s send new mode goal %s", moduleNodeName.c_str(), rexos_statemachine::mode_txt[mode]);
	rexos_statemachine::ChangeModeGoal goal;
	goal.desiredMode = mode;
	changeModeActionClient.sendGoal(goal);
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

} /* namespace equiplet_node */
