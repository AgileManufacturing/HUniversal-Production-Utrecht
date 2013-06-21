#pragma once

#include <ros/ros.h>

#include <rexos_statemachine/StateMachine.h>
#include <rexos_statemachine/Listener.h>

namespace rexos_statemachine{

class ModuleStateMachine : public StateMachine, public Listener {
	std::string moduleName;
	int moduleId;
	int equipletId;

	ros::ServiceClient changeStateNotificationClient;
	ros::ServiceClient changeModeNotificationClient;
public:
	ModuleStateMachine(std::string moduleName, int equipletId, int moduleId);
private:
	void onStateChanged();
	void onModeChanged();
};

}
