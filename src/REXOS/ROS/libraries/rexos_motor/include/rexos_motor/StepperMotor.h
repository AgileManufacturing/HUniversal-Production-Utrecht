/**
 * @file StepperMotor.h
 * @brief Steppermotor driver.
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

#define DEFAULT_MOTION_SLOT 1

#include <rexos_motor/MotorRotation.h>
#include <rexos_io/RtuModbusInputOutputController.h>
#include <rexos_motor/CRD514KD.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_motor/StepperMotorProperties.h>

namespace rexos_motor{
	class StepperMotor : public MotorInterface{
	public:
		StepperMotor(rexos_io::RtuModbusInputOutputController* ioController, uint16_t motorAddress, StepperMotorProperties properties);

		/**
		 * Deconstructor of StepperMotor. Tries to turn to power off.
		 **/
		virtual ~StepperMotor(void);

		/**
		 * If the motor is not powered on yet, try to configure the motor and turn on excitement. Also reset any alarms.
		 **/
		virtual void powerOn(void);
		/**
		 * If the motor is powered on, try to turn off excitement.
		 **/
		virtual void powerOff(void);
		
		/**
		 * Stops the motors & clears the motion queue.
		 * 
		 **/
		virtual void stop(void);

		/**
		 * Sets the motors step count to 0.
		 *
		 * @note This method requires excitement to be briefly turned off, so in real life scenarios could cause movement to go undetected.
		 **/
		virtual void resetCounter(void);

		void moveTo(const rexos_motor::MotorRotation& motorRotation, int motionSlot);
		
		virtual void writeRotationData(const rexos_motor::MotorRotation& motorRotation, bool useDeviation);
		void writeRotationData(const rexos_motor::MotorRotation& motorRotation, int motionSlot, bool useDeviation = true);
		
		/**
		 * Start the motor to move according to the set registers. Will wait for the motor to be ready before moving.
		 **/
		virtual void startMovement();
		void startMovement(int motionSlot);
		/**
		 * Wait untill the motor indicates that the end location is reached.
		 **/
		virtual void waitTillReady(void);

		
		virtual bool isReady();

		/**
		 * Sets the motor controller to incremental mode.
		 * @param motionSlot The motion slot to be set to incremental.
		 **/
		virtual void setRelativeMode();
		/**
		 * Sets the motor controller to absolute mode.
		 * @param motionSlot The motion slot to be set to absolute.
		 **/
		virtual void setAbsoluteMode();
		
		/**
		 * Sets the deviation in the motor class, and in hardware (if supported)
		 */
		virtual void setDeviation(double deviationAngle);
		
		/**
		 * Enables the limitations in the motor class, and in hardware (if supported)
		 */
		virtual void enableAngleLimitations();
		
		/**
		 * Disables the limitations in the motor class, and in hardware (if supported)
		 */
		virtual void disableAngleLimitations();
		
		/**
		 * Returns the angle per microstep, in radians.
		 * 
		 * @return The minimum angle, in radians, the StepperMotor can travel on the theoretical plane.
		 **/
		//inline double getMicroStepAngle(void) const{ return properties.microStepAngle; }
		
	private:
		StepperMotorProperties properties;
		
		/**
		 * @var ModbusController* modbus
		 * Controller for the modbus communication.
		 **/
		rexos_io::RtuModbusInputOutputController* ioController;

		/**
		 * @var Slaves::t motorAddress
		 * The address of the slave on the modbus
		 **/
		uint16_t motorAddress;

		/**
		 * Checks whether the motion slot is used. Throws an std::out_of_range exception if not.
		 * @param motionSlot the motion slot to be checked.
		 **/
		void checkMotionSlot(int motionSlot);
	};
}
