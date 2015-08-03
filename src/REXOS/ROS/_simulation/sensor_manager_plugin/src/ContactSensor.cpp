#include <sensor_manager_plugin/ContactSensor.h>

using namespace gazebo;

namespace sensor_manager_plugin {
	bool ContactSensor::isTriggered() {
		double jointAngle = joint->GetAngle(0).Radian();
		if(		(angle < 0 && jointAngle <= angle) ||
				(angle > 0 && jointAngle >= angle)) {
			return true;
		} else {
			return false;
		}
	}
}
