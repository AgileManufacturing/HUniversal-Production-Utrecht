/*
 * ModuleProxy.h
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#ifndef MODULEPROXY_H_
#define MODULEPROXY_H_

#include <actionlib/client/simple_action_client.h>

#include <bondcpp/bond.h>

#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

#include <rexos_statemachine/ChangeStateAction.h>
#include <rexos_statemachine/ChangeModeAction.h>
#include <rexos_statemachine/SetInstructionAction.h>

#include <equiplet_node/StateUpdate.h>
#include <equiplet_node/ModeUpdate.h>
#include <equiplet_node/SetInstruction.h>

#include "equiplet_node/ModuleProxyListener.h"
 
// GCC system header to suppress libjson warnings
#pragma GCC system_header
#include <libjson/libjson.h>

namespace equiplet_node {

class EquipletNode;

typedef actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction> ChangeStateActionClient;
typedef actionlib::SimpleActionClient<rexos_statemachine::ChangeModeAction> ChangeModeActionClient;
typedef actionlib::SimpleActionClient<rexos_statemachine::SetInstructionAction> SetInstructionActionClient;

class ModuleProxy {
public:
	ModuleProxy(std::string equipletNodeName, std::string moduleName, int equipletId, int moduleId, ModuleProxyListener* mpl = NULL);
	virtual ~ModuleProxy();

	void setModuleProxyListener(ModuleProxyListener* mpl);

	rexos_statemachine::State getCurrentState();
	rexos_statemachine::Mode getCurrentMode();

	int getModuleId();

	std::string getModuleNodeName();

	void changeState(rexos_statemachine::State state);
	void changeMode(rexos_statemachine::Mode mode);
	void setInstruction(std::string OID, JSONNode n);

private:
	ModuleProxy(std::string equipletNodeName, std::string moduleNodeName, ModuleProxyListener* mpl = NULL);

	bool onStateChangeServiceCallback(StateUpdateRequest &req, StateUpdateResponse &res);
	bool onModeChangeServiceCallback(ModeUpdateRequest &req, ModeUpdateResponse &res);
	void onInstructionServiceCallback(const actionlib::SimpleClientGoalState& state, const rexos_statemachine::SetInstructionResultConstPtr& result);

	int moduleId;

	ModuleProxyListener* moduleProxyListener;

	std::string moduleNodeName;

	ros::NodeHandle nodeHandle;

	ChangeStateActionClient changeStateActionClient;
	ChangeModeActionClient changeModeActionClient;
	SetInstructionActionClient setInstructionActionClient;

	ros::ServiceServer stateUpdateServiceServer;
	ros::ServiceServer modeUpdateServiceServer;
	ros::ServiceServer instructionUpdateServiceServer;

	rexos_statemachine::State currentState;
	rexos_statemachine::Mode currentMode;
	
	/**
	 * The bond to bind the module with the equiplet
	 **/
	bond::Bond* bond;
};

} /* namespace equiplet_node */
#endif /* MODULEPROXY_H_ */
