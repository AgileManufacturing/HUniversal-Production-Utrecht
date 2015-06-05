/**
 * @file MotorInterface.h
 * @brief DeltaRobot motor interface.
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Koen Braham
 * @author 1.1 Dennis Koole
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

#include <rexos_motor/MotorProperties.h>
#include <rexos_motor/MotorRotation.h>

namespace rexos_motor{
	/**
	 * Interface for the deltaronot motors.
	 **/
	class MotorInterface{
	protected:
		MotorInterface(MotorProperties properties);
		virtual ~MotorInterface(void);
		
		MotorProperties properties;
		/**
		 * The current angle the motor
		**/
		double currentAngle;
		/**
		 * The angle the motor should have after move has completed
		**/
		double targetAngle;
		/**
		 * The deviation between the motors 0 degrees and the horizontal 0 degrees.
		 **/
		double deviation;
		
		bool powerStatus;
		/**
		 * @var bool anglesLimited
		 * If hardware limitations are set on the angles the motor can travel.
		 **/
		bool anglesLimited;
		
	public:
		/**
		 * Turns on the motors.
		 **/
		virtual void powerOn(void) = 0;

		/**
		 * Shuts down the motors.
		 **/
		virtual void powerOff(void) = 0;
		
		/**
		 * Start the motors.
		 **/
		virtual void startMovement(void) = 0;
		/**
		 * Stops the motors.
		 **/
		virtual void stop(void) = 0;

		
		/**
		 * Write the rotation to be performed to the motor, but do NOT start the rotation
		 */
		virtual void writeRotationData(const rexos_motor::MotorRotation& motorRotation, bool useDeviation = true) = 0;
		
		/**
		 * Wait till the motor has finished any rotations.
		 **/
		virtual void waitTillReady(void) = 0;
		
		/**
		 * Configures the motor to use relative positioning
		 */
		virtual void setRelativeMode(void) = 0;
		
		/**
		 * Configures the motor to use absolute positioning
		 */
		virtual void setAbsoluteMode(void) = 0;
		
		/**
		 * Sets the deviation in the motor class, and in hardware (if supported)
		 */
		virtual void setDeviation(double deviationAngle) = 0;
		
		/**
		 * Enables the limitations in the motor class, and in hardware (if supported)
		 */
		virtual void enableAngleLimitations() = 0;
		
		/**
		 * Disables the limitations in the motor class, and in hardware (if supported)
		 */
		virtual void disableAngleLimitations() = 0;
		
		/**
		 * Gets the minimum angle, in radians, the motor can travel on the theoretical plane.
		 **/
		inline double getMinAngle(void) const {
			return properties.motorMinAngle;
		}

		/**
		 * Gets the maximum angle, in radians, the motor can travel on the theoretical plane.
		 **/
		inline double getMaxAngle(void) const {
			return properties.motorMaxAngle;
		}

		/**
		 * Gets the current angle of the motor in radians.
		 **/
		inline double getCurrentAngle(void) const {
			return currentAngle;
		}
		
		inline bool isPoweredOn(void) {
			return powerStatus;
		}
		
		inline bool isValidAngle(double angle) {
			return (angle >= getMinAngle() && angle <= getMaxAngle());
		}
		
		/**
		 * Store the angle that was given for a movement after the movement is done to the local variable currentAngle.
		 **/
		inline void updateAngle(void) {
			currentAngle = targetAngle;
		}
		/**
		 * Rotate the motors.
		 **/
		void moveTo(const rexos_motor::MotorRotation& motorRotation);
	};
}
