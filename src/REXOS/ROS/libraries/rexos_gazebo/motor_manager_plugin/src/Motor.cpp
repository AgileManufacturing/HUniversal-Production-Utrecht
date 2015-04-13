#include <motor_manager_plugin/Motor.h>

#include <gazebo/math/Angle.hh>
#include <ros/ros.h>
#include <limits>

namespace motor_manager_plugin {
	Motor::Motor() :
			lowerAngleLimit(-10), upperAngleLimit(10), relativeMode(false), powerStatus(false), isActive(false), 
			minSpeed(0), startAngle(0) {
		
	}
	void Motor::setPowerStatus(bool powerStatus) {
		if(powerStatus == true) {
			// power on
			double angle = joint->GetAngle(0).Radian();
			joint->SetLowStop(0, gazebo::math::Angle(angle));
			joint->SetHighStop(0, gazebo::math::Angle(angle));
		} else {
			joint->SetLowStop(0, gazebo::math::Angle(-std::numeric_limits<double>::infinity()));
			joint->SetHighStop(0, gazebo::math::Angle(std::numeric_limits<double>::infinity()));
		}
		this->powerStatus = powerStatus;
	}
	bool Motor::isMotorReady() {
		return !isActive;
	}
	void Motor::startMotor(gazebo::common::Time currentTime) {
		if(powerStatus == true) {
			ROS_INFO_STREAM("startMotor " << currentTime);
			isActive = true;
			startTime = currentTime;
			startAngle = joint->GetAngle(0).Radian();
		} else {
			ROS_FATAL_STREAM("Unable to start motor because it is not powerd on");
		}
	}
	void Motor::stopMotor(gazebo::common::Time currentTime) {
		if(isActive == true) {
			double newAngle = calculateMotorAngle(currentTime);
			// TODO what if already deccelerating?
			if(targetAngle < startAngle) {
				// rotating in negative direction
				targetAngle = newAngle - calculateAccelerationDistance(maxDecceleration);
			} else {
				// rotating in positive direction
				targetAngle = newAngle + calculateAccelerationDistance(maxDecceleration);
			}
		}
	}
	void Motor::updateMotor(gazebo::common::Time currentTime) {
		if(isActive == true) {
			double newAngle = calculateMotorAngle(currentTime);
			setAngle(newAngle);
		}
	}
	double Motor::calculateMotorAngle(gazebo::common::Time currentTime) {
			ROS_INFO_STREAM("-----------------------------");
		// motor is rotating
		double deltaTime = (currentTime - startTime).Double();
			ROS_INFO_STREAM("deltaTime " << deltaTime);
		double accelerationDuration = calculateAccelerationDuration(maxAcceleration);
			ROS_INFO_STREAM("accelerationDuration " << accelerationDuration);
		double deccelerationDuration = calculateAccelerationDuration(maxDecceleration);
			ROS_INFO_STREAM("deccelerationDuration " << deccelerationDuration);
		
		double accelerationDistance = calculateAccelerationDistance(maxAcceleration);
			ROS_INFO_STREAM("accelerationDistance " << accelerationDistance);
		double deccelerationDistance = calculateAccelerationDistance(maxDecceleration);
			ROS_INFO_STREAM("deccelerationDistance " << deccelerationDistance);
		double totalDistance = std::abs(targetAngle - startAngle);
			ROS_INFO_STREAM("totalDistance " << totalDistance);
		double absoluteTotalDistance = std::abs(targetAngle - startAngle);
		double speedDifference = maxSpeed - minSpeed;
		
		double newAngle;
		
		if(accelerationDistance + deccelerationDistance > absoluteTotalDistance) {
			// this is a two phase motion. It has no constant speed phase and the acceleration phase is interrupted by the decelleration phase
			
			// determine the point where de acceleration is interrupted by the decelleration
			double reductionFactor = absoluteTotalDistance / (accelerationDistance + deccelerationDistance);
			ROS_INFO_STREAM("reductionFactor " << reductionFactor);
			double turnoverPoint = accelerationDistance * reductionFactor;
			ROS_INFO_STREAM("turnoverPoint " << turnoverPoint);
			double reducedAccelerationTime = std::sqrt((accelerationDistance * reductionFactor - minSpeed) / (maxAcceleration / 2));
			ROS_INFO_STREAM("reducedAccelerationTime " << reducedAccelerationTime);
			double reducedDeccelerationTime = std::sqrt((deccelerationDistance * reductionFactor - minSpeed) / (maxDecceleration / 2));
			ROS_INFO_STREAM("reducedDeccelerationTime " << reducedDeccelerationTime);
			double totalTravelTime = reducedAccelerationTime + reducedDeccelerationTime;
			ROS_INFO_STREAM("totalTravelTime " << totalTravelTime);
			
			if(deltaTime >= totalTravelTime) {
			ROS_INFO_STREAM("2p stop");
				// motor is at destination
				newAngle = targetAngle;
				isActive = false;
			} else if(deltaTime < reducedAccelerationTime) {
			ROS_INFO_STREAM("2p acc");
				// motor is accelerating
				double averageSpeed = minSpeed + maxAcceleration * (deltaTime / 2);
			ROS_INFO_STREAM("averageSpeed " << averageSpeed);
				if(targetAngle < startAngle) {
					// rotating in negative direction
					newAngle = startAngle - averageSpeed * deltaTime;
				} else {
					// rotating in positive direction
					newAngle = startAngle + averageSpeed * deltaTime;
				}
			} else {
			ROS_INFO_STREAM("2p decc");
				// motor is decellerating
				double averageSpeed = minSpeed + maxDecceleration * ((totalTravelTime - deltaTime) / 2);
			ROS_INFO_STREAM("averageSpeed " << averageSpeed);
				if(targetAngle < startAngle) {
					// rotating in negative direction
					newAngle = targetAngle + averageSpeed * (totalTravelTime - deltaTime);
				} else {
					// rotating in positive direction
					newAngle = targetAngle - averageSpeed * (totalTravelTime - deltaTime);
				}
			}
		} else {
			// this is a three phase motion
			double distanceInConstantSpeedPhase = absoluteTotalDistance - accelerationDistance - deccelerationDistance;
			double totalTravelTime = accelerationDuration + (distanceInConstantSpeedPhase / maxSpeed) + deccelerationDuration;
			if(deltaTime >= totalTravelTime) {
			ROS_INFO_STREAM("3p stop");
				// motor is at destination
				newAngle = targetAngle;
				isActive = false;
			}
			else if(deltaTime < accelerationDuration) {
			ROS_INFO_STREAM("3p acc");
				// motor is accelerating
				double averageSpeed = minSpeed + maxAcceleration * (deltaTime / 2);
				if(targetAngle < startAngle) {
					// rotating in negative direction
					newAngle = startAngle - averageSpeed * deltaTime;
				} else {
					// rotating in positive direction
					newAngle = startAngle + averageSpeed * deltaTime;
				}
			} else if(deltaTime >= totalTravelTime - deccelerationDuration) {
			ROS_INFO_STREAM("3p decc");
				// motor is decelerating
				double averageSpeed = minSpeed + maxDecceleration * ((totalTravelTime - deltaTime) / 2);
				if(targetAngle < startAngle) {
					// rotating in negative direction
					newAngle = targetAngle + averageSpeed * (totalTravelTime - accelerationDuration - deltaTime);
				} else {
					// rotating in positive direction
					newAngle = targetAngle - averageSpeed * (totalTravelTime - accelerationDuration - deltaTime);
				}
			} else {
			ROS_INFO_STREAM("3p cons");
				// motor is at constant speed
				double averageSpeed = maxSpeed;
				if(targetAngle < startAngle) {
					// rotating in negative direction
					newAngle = startAngle + accelerationDistance + averageSpeed * (totalTravelTime - accelerationDuration - deltaTime);
				} else {
					// rotating in positive direction
					newAngle = startAngle - (accelerationDistance + averageSpeed * (totalTravelTime - accelerationDuration - deltaTime));
				}
			}
		}
		return newAngle;
	}
	
	void Motor::setAngle(double angle) {
		// clip to the limits
		if(angle < lowerAngleLimit) {
			angle = lowerAngleLimit;
		} else if(angle > upperAngleLimit) {
			angle = upperAngleLimit;
		}
		joint->SetLowStop(0, gazebo::math::Angle(angle));
		joint->SetHighStop(0, gazebo::math::Angle(angle));
	}
	double Motor::calculateAccelerationDuration(double acceleration) {
		double speedDifference = maxSpeed - minSpeed;
		return speedDifference / acceleration;
	}
	double Motor::calculateAccelerationDistance(double acceleration) {
		double speedDifference = maxSpeed - minSpeed;
		double timeRequiredToAccelerate = calculateAccelerationDuration(acceleration);
		double averageSpeedDuringSAcceleration = minSpeed + speedDifference / 2;
		return timeRequiredToAccelerate * averageSpeedDuringSAcceleration;
	}
}
