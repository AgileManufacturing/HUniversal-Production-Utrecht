#pragma once

#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Time.hh>

namespace motor_manager_plugin {
	class Motor {
	public: 
		gazebo::physics::JointPtr joint;
		
		double lowerAngleLimit;
		double upperAngleLimit;
		bool relativeMode;
		
		double minSpeed;
		double maxSpeed;
		double maxAcceleration;
		double maxDecceleration;
		double targetAngle;
		
		double powerOnAngle;
		
		Motor();
		void setPowerStatus(bool powerStatus);
		bool isMotorReady();
		void startMotor(gazebo::common::Time currentTime);
		void stopMotor(gazebo::common::Time currentTime);
		void updateMotor(gazebo::common::Time currentTime);
	protected:
		gazebo::common::Time currentMoveStartTime;
		double currentMoveMinSpeed;
		double currentMoveMaxSpeed;
		double currentMoveMaxAcceleration;
		double currentMoveMaxDecceleration;
		double currentMoveStartAngle;
		double currentMoveTargetAngle;
		
		bool powerStatus;
		bool isActive;
		
		void setAngle(double angle);
		double calculateMotorAngle(gazebo::common::Time currentTime);
		double calculateAccelerationDuration(double acceleration);
		double calculateAccelerationDistance(double acceleration);
	};
}