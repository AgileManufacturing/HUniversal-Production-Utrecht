#pragma once

#include <ros/ros.h>

#include <rexos_statemachine/StateMachine.h>
#include <rexos_statemachine/Listener.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include "rexos_logger/rexos_logger.h"

namespace rexos_statemachine{

class ModuleStateMachine : public StateMachine {
private:
	std::string equipletName;
	rexos_datatypes::ModuleIdentifier moduleIdentifier;

	ros::ServiceClient changeStateNotificationClient;
	ros::ServiceClient changeModeNotificationClient;
public:
	ModuleStateMachine(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool actorModule);
	~ModuleStateMachine();
protected:
	virtual void onStateChanged(rexos_statemachine::State state);
	virtual void onModeChanged(rexos_statemachine::Mode mode);

	void setInError();

	bool actorModule;
};

}
