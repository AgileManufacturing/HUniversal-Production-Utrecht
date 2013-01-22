/**
 * @file MotorManager.h
 * @brief Motor management for concurrent movement.
 * @date Created: 2012-10-02
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

#include <rexos_modbus/ModbusController.h>
#include <rexos_motor/StepperMotor.h>

namespace rexos_motor{

	/**
	 * Motor management for concurrent movement.
	 **/
	class MotorManager{
	public:
		/**
		 * Constructor for the motor manager
		 *
		 * @param modbus Pointer to an established modbus connection.
		 * @param motors Pointer array containing all motors for this manager.
		 * @param numberOfMotors Number of motors in the pointer array.
		 **/
		MotorManager(rexos_modbus::ModbusController* modbus, StepperMotor** motors, int numberOfMotors) :
			modbus(modbus), motors(motors), numberOfMotors(numberOfMotors), poweredOn(false){}

		void powerOn(void);
		void powerOff(void);

		/**
		 * Check whether the motormanager has been initiated.
		 * @return bool PowerOn state.
		 **/
		bool isPoweredOn(void){ return poweredOn; }
		void startMovement(int motionSlot);

	private:
		/**
		 * @var ModbusController::ModbusController* modbus
		 * Pointer to an established modbus connection.
		 **/
		rexos_modbus::ModbusController* modbus;

		/**
		 * @var StepperMotor** motors
		 * Pointer array containing all motors for this manager.
		 **/
		StepperMotor** motors;

		/**
		 * @var int numberOfMotors
		 * Number of motors in the pointer array.
		 **/
		int numberOfMotors;

		/**
		 * @var bool poweredOn
		 * Stores whether the motor manager has been turned on.
		 **/
		bool poweredOn;
	};
}
