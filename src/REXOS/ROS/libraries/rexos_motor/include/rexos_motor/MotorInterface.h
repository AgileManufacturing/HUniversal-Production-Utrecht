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

#include <rexos_datatypes/MotorRotation.h>

namespace rexos_motor{
	/**
	 * Interface for the deltaronot motors.
	 **/
	class MotorInterface{
	protected:
		MotorInterface(void){}
			 
	public:
		virtual ~MotorInterface(void){}

		/**
		 * Turns on the motors.
		 **/
		virtual void powerOn(void) = 0;

		/**
		 * Shuts down the motors.
		 **/
		virtual void powerOff(void) = 0;

		/**
		 * Stops the motors.
		 **/
		virtual void stop(void) = 0;

		/**
		 * Rotate the motors.
		 * 
		 * @param motorRotation Defines the angles, speed, acceleration and deceleration of the motors.
		 **/
		virtual void moveTo(const rexos_datatypes::MotorRotation& motorRotation) = 0;

		/**
		 * Wait till the motor has finished any rotations.
		 **/
		void waitTillReady(void);

		/**
		 * Get the minimal angle the motors can move to.
		 * 
		 * @return angle in radians.
		 **/
		virtual double getMinAngle(void) const = 0;

		/**
		 * Get the maximum angle the motors can move to.
		 * 
		 * @return angle in radians.
		 **/
		virtual double getMaxAngle(void) const = 0;

		/**
		 * Sets the current angle.
		 **/
		virtual void setCurrentAngle(double angle) = 0;

		/**
		 * Determine if the motor driver(s) are powered on.
		 * 
		 * @return true of powered on, false otherwise.
		 **/
		virtual bool isPoweredOn(void) = 0;
	};
}
