#pragma once

#include <ros/ros.h>

#include <rexos_bond/Bond.h>

#include <rexos_statemachine/StateMachine.h>
#include <rexos_statemachine/Listener.h>
#include <rexos_knowledge_database/ModuleIdentifier.h>

namespace rexos_statemachine{

class ModuleStateMachine : public StateMachine, public rexos_bond::BondListener {
	std::string equipletName;
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier;

	ros::ServiceClient changeStateNotificationClient;
	ros::ServiceClient changeModeNotificationClient;
public:
	ModuleStateMachine(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier, bool actorModule);
	~ModuleStateMachine();
protected:
	virtual void onStateChanged(rexos_statemachine::State state);
	virtual void onModeChanged(rexos_statemachine::Mode mode);

	void setInError();

	bool actorModule;
	TransitionActionClient transitionActionClient;

private:
	/**
	 * The bond to bind the module with the equiplet
	 **/
	rexos_bond::Bond* bond;
protected:
	virtual void onBondCallback(rexos_bond::Bond* bond, Event event);
};

}
