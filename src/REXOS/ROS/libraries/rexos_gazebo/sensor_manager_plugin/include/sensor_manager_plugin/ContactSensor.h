#pragma once

#include <gazebo/physics/Joint.hh>
using namespace gazebo;

namespace sensor_manager_plugin {
	class ContactSensor {
	public: 
		physics::JointPtr joint;
		double angle;
	public: 
		bool isTriggered();
	};
}
