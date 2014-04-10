#pragma once

#include <ros/ros.h>

#include <rexos_bond/Bond.h>

#include <rexos_statemachine/StateMachine.h>
#include <rexos_statemachine/Listener.h>
#include <rexos_knowledge_database/ModuleIdentifier.h>

namespace rexos_statemachine{

class ModuleStateMachine : public StateMachine, public Listener, public rexos_bond::BondListener {
	std::string equipletName;
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier;

	ros::ServiceClient changeStateNotificationClient;
	ros::ServiceClient changeModeNotificationClient;
public:
	ModuleStateMachine(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier, bool actorModule);
	~ModuleStateMachine();
protected:
	virtual void onStateChanged();
	virtual void onModeChanged();

	void setInError();

	bool actorModule;

private:
	/**
	 * The bond to bind the module with the equiplet
	 **/
	rexos_bond::Bond* bond;
protected:
	virtual void onBondCallback(rexos_bond::Bond* bond, Event event);
};

}
