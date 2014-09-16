/*
 * ModuleProxy.h
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#ifndef MODULEPROXY_H_
#define MODULEPROXY_H_

#include <actionlib/client/simple_action_client.h>

#include <rexos_bond/Bond.h>

#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

#include <rexos_statemachine/StateMachine.h>
#include <rexos_statemachine/ChangeStateAction.h>
#include <rexos_statemachine/ChangeModeAction.h>
#include <rexos_statemachine/SetInstructionAction.h>

#include <rexos_knowledge_database/ModuleIdentifier.h>

#include <equiplet_node/StateUpdate.h>
#include <equiplet_node/ModeUpdate.h>
#include <equiplet_node/SetInstruction.h>

#include "equiplet_node/ModuleProxyListener.h"
#include "rexos_logger/rexos_logger.h"

// GCC system header to suppress libjson warnings
#pragma GCC system_header
#include <libjson/libjson.h>

namespace equiplet_node {

class EquipletNode;

typedef actionlib::SimpleActionClient<rexos_statemachine::ChangeStateAction> ChangeStateActionClient;
typedef actionlib::SimpleActionClient<rexos_statemachine::ChangeModeAction> ChangeModeActionClient;
typedef actionlib::SimpleActionClient<rexos_statemachine::SetInstructionAction> SetInstructionActionClient;

class ModuleProxy : public rexos_bond::BondListener{
public:
	ModuleProxy(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier, ModuleProxyListener* mpl = NULL);
	virtual ~ModuleProxy();

	void setModuleProxyListener(ModuleProxyListener* mpl);

	rexos_statemachine::State getCurrentState();
	rexos_statemachine::Mode getCurrentMode();

	rexos_knowledge_database::ModuleIdentifier getModuleIdentifier();

	void changeState(rexos_statemachine::State state);
	void changeMode(rexos_statemachine::Mode mode);
	void setInstruction(std::string OID, JSONNode n);
	void goToNextTransitionPhase();

private:
	std::string moduleNamespaceName;
	std::string equipletNamespaceName;
	
	bool onStateChangeServiceCallback(StateUpdateRequest &req, StateUpdateResponse &res);
	bool onModeChangeServiceCallback(ModeUpdateRequest &req, ModeUpdateResponse &res);
	void onInstructionServiceCallback(const actionlib::SimpleClientGoalState& state, const rexos_statemachine::SetInstructionResultConstPtr& result);
	void onModuleTransitionGoalCallback(const rexos_statemachine::TransitionGoalConstPtr& feedback);

	rexos_knowledge_database::ModuleIdentifier moduleIdentifier;

	ModuleProxyListener* moduleProxyListener;
	
	ros::NodeHandle nodeHandle;

	ChangeStateActionClient changeStateActionClient;
	ChangeModeActionClient changeModeActionClient;
	SetInstructionActionClient setInstructionActionClient;

	ros::ServiceServer stateUpdateServiceServer;
	ros::ServiceServer modeUpdateServiceServer;
	ros::ServiceServer instructionUpdateServiceServer;
	rexos_statemachine::TransitionActionServer transitionActionServer;
	
	rexos_statemachine::State currentState;
	rexos_statemachine::State desiredState;
	rexos_statemachine::Mode currentMode;
	
	boost::condition_variable nodeStartupCondition;
	boost::mutex nodeStartupMutex;
	boost::condition_variable transitionPhaseCondition;
	boost::mutex transitionPhaseMutex;
	bool allowedToContinue;
	
	bool connectedWithNode;
	/**
	 * The bond to bind the module with the equiplet
	 **/
	rexos_bond::Bond* bond;
protected:
	virtual void onBondCallback(rexos_bond::Bond* bond, Event event);
public:
	void bind();
};

} /* namespace equiplet_node */
#endif /* MODULEPROXY_H_ */
