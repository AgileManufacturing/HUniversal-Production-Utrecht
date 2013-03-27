/**
 * @file MotorRotation.h
 * @brief Template class for rotation data for the motor.
 * @date Created: 2012-10-03
 *
 * @author Koen Braham
 * @author Dennis Koole
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

namespace rexos_datatypes{
	/**
	 * Data entity for rotation data for the motor.
	 **/
	class MotorRotation{
	public:
		/**
		 * @var double angle
		 * The angle in radians the motor has to travel towards
		 **/
		double angle;

		/**
		 * @var double speed
		 * The speed in radians per second.
		 **/
		double speed;

		/**
		 * @var double acceleration
		 * The acceleration in radians per second per second.\n 
		 * Minimum: 1.256637061 rad\n
		 * Maximum: 1256637.061 rad
		 **/
		double acceleration;

		/**
		 * @var double deceleration
		 * The deceleration in radians per second per second.\n 
		 * Minimum: 1.256637061 rad\n
		 * Maximum: 1256637.061 rad
		 **/
		double deceleration;

		/**
		 * Instantiation of rotation data for the motor.
		 *
		 * @param angle The angle in radians the motor has to travel towards. Defaults to 0.
		 * @param speed The speed in radians per second. Defaults to 1.
		 * @param acceleration The acceleration in radians per second per second. Defaults to 10.
		 * @param deceleration The deceleration in radians per second per second. Defaults to 10.
		 **/
		MotorRotation(double angle = 0, double speed = 1, double acceleration = 10, double deceleration = 10) : angle(angle), speed(speed), acceleration(acceleration), deceleration(deceleration){}
	};
}
