/*
 * ModuleProxy.h
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#ifndef MODULEPROXY_H_
#define MODULEPROXY_H_

#include <actionlib/client/simple_action_client.h>

#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

#include <rexos_statemachine/ChangeStateAction.h>
#include <rexos_statemachine/ChangeModeAction.h>

#include <equiplet_node/StateUpdate.h>
#include <equiplet_node/ModeUpdate.h>

#include "equiplet_node/ModuleProxyListener.h"

namespace equiplet_node {

class EquipletNode;

typedef actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction> ChangeStateActionClient;
typedef actionlib::SimpleActionClient<rexos_statemachine::ChangeModeAction> ChangeModeActionClient;

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

	bool setModuleInstruction();

private:
	ModuleProxy(std::string equipletNodeName, std::string moduleNodeName, ModuleProxyListener* mpl = NULL);
	bool onStateChangeServiceCallback(StateUpdateRequest &req, StateUpdateResponse &res);
	bool onModeChangeServiceCallback(ModeUpdateRequest &req, ModeUpdateResponse &res);

	int moduleId;

	ModuleProxyListener* moduleProxyListener;

	std::string moduleNodeName;

	ros::NodeHandle nodeHandle;

	ChangeStateActionClient changeStateActionClient;
	ChangeModeActionClient changeModeActionClient;
	ros::ServiceServer stateUpdateServiceServer;
	ros::ServiceServer modeUpdateServiceServer;

	rexos_statemachine::State currentState;
	rexos_statemachine::Mode currentMode;
};

} /* namespace equiplet_node */
#endif /* MODULEPROXY_H_ */
