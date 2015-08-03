#include <motor_manager_plugin/Motor.h>

#include <gazebo/math/Angle.hh>
#include <ros/ros.h>
#include <limits>

namespace motor_manager_plugin {
	Motor::Motor() :
			lowerAngleLimit(NAN), upperAngleLimit(NAN), relativeMode(false), minSpeed(NAN), powerStatus(false), isActive(false) {
		
	}
	void Motor::setPowerStatus(bool powerStatus) {
		if(powerStatus == true) {
			// power on
			double angle = joint->GetAngle(0).Radian();
			joint->SetLowStop(0, gazebo::math::Angle(angle));
			joint->SetHighStop(0, gazebo::math::Angle(angle));
			powerOnAngle = angle;
			currentMoveTargetAngle = powerOnAngle;
			isActive = false;
		} else {
			joint->SetLowStop(0, gazebo::math::Angle(-std::numeric_limits<double>::infinity()));
			joint->SetHighStop(0, gazebo::math::Angle(std::numeric_limits<double>::infinity()));
			isActive = false;
		}
		this->powerStatus = powerStatus;
	}
	bool Motor::isMotorReady() {
		return !isActive;
	}
	void Motor::startMotor(gazebo::common::Time currentTime) {
		if(powerStatus == true) {
//			ROS_INFO_STREAM("startMotor " << currentTime);
//			ROS_INFO_STREAM("targetAngle " << targetAngle);
			isActive = true;

			currentMoveMinSpeed = minSpeed;
			currentMoveMaxSpeed = maxSpeed;
			currentMoveMaxAcceleration = maxAcceleration;
			currentMoveMaxDecceleration = maxDecceleration;
			currentMoveStartTime = currentTime;
			currentMoveStartAngle = currentMoveTargetAngle;

			if(relativeMode == true) {
				// relative mode
				currentMoveTargetAngle = currentMoveStartAngle + targetAngle;
			} else {
				// absolute mode
				currentMoveTargetAngle = targetAngle + powerOnAngle;
			}
//			ROS_INFO_STREAM("currentMoveStartAngle " << currentMoveStartAngle);
//			ROS_INFO_STREAM("currentMoveTargetAngle " << currentMoveTargetAngle);
		} else {
			ROS_ERROR_STREAM("Unable to start motor because it is not powerd on");
		}
	}
	void Motor::stopMotor(gazebo::common::Time currentTime) {
		if(isActive == true) {
			double newAngle = calculateMotorAngle(currentTime);
			// TODO what if already deccelerating?
			if(currentMoveTargetAngle < currentMoveStartAngle) {
				// rotating in negative direction
				currentMoveTargetAngle = newAngle - calculateAccelerationDistance(currentMoveMaxDecceleration);
			} else {
				// rotating in positive direction
				currentMoveTargetAngle = newAngle + calculateAccelerationDistance(currentMoveMaxDecceleration);
			}
		}
	}
	void Motor::updateMotor(gazebo::common::Time currentTime) {
		if(isActive == true) {
			double newAngle = calculateMotorAngle(currentTime);
//			ROS_INFO_STREAM("newAngle " << newAngle);
			setAngle(newAngle);
		}
	}
	double Motor::calculateMotorAngle(gazebo::common::Time currentTime) {
//		ROS_INFO_STREAM("-----------------------------");
		// motor is rotating
		double deltaTime = (currentTime - currentMoveStartTime).Double();
		double accelerationDuration = calculateAccelerationDuration(currentMoveMaxAcceleration);
		double deccelerationDuration = calculateAccelerationDuration(currentMoveMaxDecceleration);
		
		double accelerationDistance = calculateAccelerationDistance(currentMoveMaxAcceleration);
		double deccelerationDistance = calculateAccelerationDistance(currentMoveMaxDecceleration);
		double totalDistance = currentMoveTargetAngle - currentMoveStartAngle;
		double absoluteTotalDistance = std::abs(totalDistance);
		
		double newAngle;
		
		if(accelerationDistance + deccelerationDistance > absoluteTotalDistance) {
			// this is a two phase motion. It has no constant speed phase and the acceleration phase is interrupted by the decelleration phase
			
			// determine the point where de acceleration is interrupted by the decelleration
//			ROS_INFO_STREAM("absoluteTotalDistance " << absoluteTotalDistance);
//			ROS_INFO_STREAM("accelerationDistance " << accelerationDistance);
//			ROS_INFO_STREAM("deccelerationDistance " << deccelerationDistance);
			double reductionFactor = absoluteTotalDistance / (accelerationDistance + deccelerationDistance);
//			ROS_INFO_STREAM("reductionFactor " << reductionFactor);
//			double turnoverPoint = accelerationDistance * reductionFactor;
//			ROS_INFO_STREAM("turnoverPoint " << turnoverPoint);
			// ABC formula
			double a, b, c;
			a = currentMoveMaxAcceleration / 2;
			b = currentMoveMinSpeed;
			c = -accelerationDistance * reductionFactor;
			double reducedAccelerationTime = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
//			ROS_INFO_STREAM("reducedAccelerationTime " << reducedAccelerationTime);
			a = currentMoveMaxDecceleration / 2;
			b = currentMoveMinSpeed;
			c = -deccelerationDistance * reductionFactor;
			double reducedDeccelerationTime = (-b + std::sqrt(b * b - 4 * a * c)) / (2 * a);
//			ROS_INFO_STREAM("reducedDeccelerationTime " << reducedDeccelerationTime);
			double totalTravelTime = reducedAccelerationTime + reducedDeccelerationTime;
//			ROS_INFO_STREAM("totalTravelTime " << totalTravelTime);
			
			if(deltaTime >= totalTravelTime) {
//			ROS_INFO_STREAM("2p stop");
				// motor is at destination
				newAngle = currentMoveTargetAngle;
				isActive = false;
			} else if(deltaTime < reducedAccelerationTime) {
//				ROS_INFO_STREAM("2p acc");
				// motor is accelerating
				double averageSpeed = currentMoveMinSpeed + currentMoveMaxAcceleration * (deltaTime / 2);
//				ROS_INFO_STREAM("averageSpeed " << averageSpeed);
				if(currentMoveTargetAngle < currentMoveStartAngle) {
					// rotating in negative direction
					newAngle = currentMoveStartAngle - averageSpeed * deltaTime;
				} else {
					// rotating in positive direction
					newAngle = currentMoveStartAngle + averageSpeed * deltaTime;
				}
			} else {
//				ROS_INFO_STREAM("2p decc");
				// motor is decellerating
				double averageSpeed = currentMoveMinSpeed + currentMoveMaxDecceleration * ((totalTravelTime - deltaTime) / 2);
//				ROS_INFO_STREAM("averageSpeed " << averageSpeed);
				if(currentMoveTargetAngle < currentMoveStartAngle) {
					// rotating in negative direction
					newAngle = currentMoveTargetAngle + averageSpeed * (totalTravelTime - deltaTime);
				} else {
					// rotating in positive direction
					newAngle = currentMoveTargetAngle - averageSpeed * (totalTravelTime - deltaTime);
				}
			}
		} else {
			// this is a three phase motion
			double distanceInConstantSpeedPhase = absoluteTotalDistance - accelerationDistance - deccelerationDistance;
			double totalTravelTime = accelerationDuration + (distanceInConstantSpeedPhase / currentMoveMaxSpeed) + deccelerationDuration;
			if(deltaTime >= totalTravelTime) {
//				ROS_INFO_STREAM("3p stop");
				// motor is at destination
				newAngle = currentMoveTargetAngle;
				isActive = false;
			}
			else if(deltaTime < accelerationDuration) {
//				ROS_INFO_STREAM("3p acc");
				// motor is accelerating
				double averageSpeed = currentMoveMinSpeed + currentMoveMaxAcceleration * (deltaTime / 2);
				if(currentMoveTargetAngle < currentMoveStartAngle) {
					// rotating in negative direction
					newAngle = currentMoveStartAngle - averageSpeed * deltaTime;
				} else {
					// rotating in positive direction
					newAngle = currentMoveStartAngle + averageSpeed * deltaTime;
				}
			} else if(deltaTime >= totalTravelTime - deccelerationDuration) {
//				ROS_INFO_STREAM("3p decc");
				// motor is decelerating
				double averageSpeed = currentMoveMinSpeed + currentMoveMaxDecceleration * ((totalTravelTime - deltaTime) / 2);
				if(currentMoveTargetAngle < currentMoveStartAngle) {
					// rotating in negative direction
					newAngle = currentMoveTargetAngle + averageSpeed * (totalTravelTime - deltaTime);
				} else {
					// rotating in positive direction
					newAngle = currentMoveTargetAngle - averageSpeed * (totalTravelTime - deltaTime);
				}
			} else {
//				ROS_INFO_STREAM("3p cons");
				// motor is at constant speed
				if(currentMoveTargetAngle < currentMoveStartAngle) {
					// rotating in negative direction
					newAngle = currentMoveStartAngle - accelerationDistance - currentMoveMaxSpeed * (deltaTime - accelerationDuration);
				} else {
					// rotating in positive direction
					newAngle = currentMoveStartAngle + accelerationDistance + currentMoveMaxSpeed * (deltaTime - accelerationDuration);
				}
			}
		}
		return newAngle;
	}
	
	void Motor::setAngle(double angle) {
		// clip to the limits
		if(angle < lowerAngleLimit) {
			angle = lowerAngleLimit;
			ROS_WARN_STREAM("Clipping to lower limit " << lowerAngleLimit << " " << angle);
		} else if(angle > upperAngleLimit) {
			angle = upperAngleLimit;
			ROS_WARN("Clipping to upper limit");
		}
		joint->SetLowStop(0, gazebo::math::Angle(angle));
		joint->SetHighStop(0, gazebo::math::Angle(angle));
	}
	double Motor::calculateAccelerationDuration(double acceleration) {
		double speedDifference = currentMoveMaxSpeed - currentMoveMinSpeed;
		return speedDifference / acceleration;
	}
	double Motor::calculateAccelerationDistance(double acceleration) {
		double speedDifference = currentMoveMaxSpeed - currentMoveMinSpeed;
		double timeRequiredToAccelerate = calculateAccelerationDuration(acceleration);
		double averageSpeedDuringSAcceleration = currentMoveMinSpeed + speedDifference / 2;
		return timeRequiredToAccelerate * averageSpeedDuringSAcceleration;
	}
}
