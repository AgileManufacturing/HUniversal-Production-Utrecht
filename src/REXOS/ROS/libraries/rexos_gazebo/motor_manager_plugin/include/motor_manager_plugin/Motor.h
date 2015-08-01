/**
 * @file Motor.h
 * @brief Motors control the angle of a joint to simulate a stepper motor. The motors use the acceleration behaviour of the CRD514-KD motor controllers.
 * These motor controllers use a constant acceleration and decceleration phase, and a constant speed phase:
 * \verbatim
 *     constant speed
 * v|       ___
 *  |      /   \
 *  | acc /     \ decc
 *  |    /       \
 *  |   /         \
 *  |  /           \
 *  | /             \
 *  |/               \
 *  |----------------- t
 * 
 *     constant speed
 * a| acc         decc
 *  |-------|
 *  |       |
 *  |----------------- t
 *  |           |
 *  |           |-----
 * 
 *     constant speed
 * s| acc         decc
 *  |             .--'
 *  |          .-'
 *  |         /
 *  |        /
 *  |       /
 *  |    .-'
 *  |.--'
 *  |----------------- t
 * \endverbatim
 * @date Created: 2015-06-00
 *
 * @author Tommas Bakker
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

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
		
		/**
		 * The min speed (in radians / second) supported by this motor. This is the speed of the motor just after it starts or just before it stops 
		 */
		double minSpeed;
		/**
		 * The max speed (in radians / second) supported by this motor. This is the speed of the motor rotates at when in the constant speed phase
		 */
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
		/**
		 * This method will calculate the duration (in seconds) of the acceleration to accelerate from min speed to max speed
		 */
		double calculateAccelerationDuration(double acceleration);
		/**
		 * This method will calculate the distance (in radians) covered during the acceleration to accelerate from min speed to max speed
		 */
		double calculateAccelerationDistance(double acceleration);
	};
}
