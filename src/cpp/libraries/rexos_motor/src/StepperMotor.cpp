/**
 * @file StepperMotor.cpp
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

#include <rexos_motor/StepperMotor.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <rexos_utilities/Utilities.h>
#include <rexos_motor/CRD514KD.h>
#include <rexos_motor/CRD514KDException.h>
#include <rexos_motor/MotorException.h>

namespace rexos_motor {
	/**
	 * Constructor of StepperMotor. Sets angles to unlimited.
	 *
	 * @param modbusController Controller for the modbus communication.
	 * @param motorIndex Index of the motor from 0 to N dependant on the amount of motors.
	 * @param minAngle Minimum for the angle, in radians, the StepperMotor can travel on the theoretical plane.
	 * @param maxAngle Maximum for the angle, in radians, the StepperMotor can travel on the theoretical plane.
	 **/
	StepperMotor::StepperMotor(rexos_modbus::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex, double minAngle, double maxAngle):
		MotorInterface(), currentAngle(0), setAngle(0), deviation(0), minAngle(minAngle), maxAngle(maxAngle), modbus(modbusController), motorIndex(motorIndex), anglesLimited(true), poweredOn(false){}

	/**
	 * Deconstructor of StepperMotor. Tries to turn to power off.
	 **/
	StepperMotor::~StepperMotor(void){
		try{
			powerOff();
		} catch(std::runtime_error& err){}
	}

	/**
	 * If the motor is not powered on yet, try to configure the motor and turn on excitement.
	 **/
	void StepperMotor::powerOn(void){
		if(!poweredOn){
			//Reset alarm
			modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 0);
			modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 1);
			modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 0);

			// Set operating modes
			modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);

			// set modes for all used motion slots
			for(int i = 0; i < CRD514KD::MOTION_SLOTS_USED; i++){
				modbus->writeU16(motorIndex, CRD514KD::Registers::OP_POSMODE + i, 1);
				modbus->writeU16(motorIndex, CRD514KD::Registers::OP_OPMODE + i, 0);
				modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SEQ_MODE + i, 1);
			}

			// Excite motor
			modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
			
			// Set motor limits
			modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((maxAngle - deviation) / CRD514KD::MOTOR_STEP_ANGLE));
			modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((minAngle - deviation) / CRD514KD::MOTOR_STEP_ANGLE));
			modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_START_SPEED, 1);
			
			// Clear counter
			modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 1);
			modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 0);

			currentAngle = 0;
			poweredOn = true;
		}
	}

	/**
	 * If the motor is powered on, try to turn off excitement.
	 **/
	void StepperMotor::powerOff(void){
		if(poweredOn){
			stop();
			modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);
			poweredOn = false;
		}
	}

	/**
	 * Stops the motors & clears the motion queue.
	 * 
	 * @note (un)locks queue_mutex.
	 **/
	void StepperMotor::stop(void){
		if(!poweredOn){
			throw MotorException("motor drivers are not powered on");
		}
		
		try{
			modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::STOP);
			modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
		} catch(rexos_modbus::ModbusException& exception){
			std::cerr << "steppermotor::stop failed: " << std::endl << "what(): " << exception.what() << std::endl;
		}
	}

	/**
	 * Sets the motors step count to 0.
	 *
	 * @note This method requires excitement to be briefly turned off, so in real life scenarios could cause movement to go undetected.
	 **/
	void StepperMotor::resetCounter(void){
		waitTillReady();
		modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);

		// Clear counter.
		modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 1);
		modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 0);
		modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
	}

	/**
	 * Moves the motor to the given position.
	 *
	 * @param motorRotation The rotational data for the motor.
	 * @param motionSlot the motion slot to be used.
	 **/
	void StepperMotor::moveTo(const rexos_datatypes::MotorRotation& motorRotation, int motionSlot){
		checkMotionSlot(motionSlot);

		writeRotationData(motorRotation, motionSlot);
		startMovement(motionSlot);
	}

	/**
	 * Moves the motor to the given position. Uses motion slot 1.
	 *
	 * @param motorRotation The rotational data for the motor.
	 **/
	void StepperMotor::moveTo(const rexos_datatypes::MotorRotation& motorRotation){
		moveTo(motorRotation, 1);
	}

	/**
	 * Writes the rotation data into the motor controller.
	 * 
	 * @param motorRotation A MotorRotation.
	 * @param motionSlot the motion slot to be written to
	 * @param useDeviation Sets whether or not to use the deviation. Defaults to true.
	 **/
	void StepperMotor::writeRotationData(const rexos_datatypes::MotorRotation& motorRotation, int motionSlot, bool useDeviation){
		if(!poweredOn){
			throw MotorException("motor drivers are not powered on");
		}

		checkMotionSlot(motionSlot);

		if(anglesLimited && (motorRotation.angle <= minAngle || motorRotation.angle >= maxAngle)){
			throw std::out_of_range("one or more angles out of range");
		}

		if(motorRotation.acceleration > CRD514KD::MOTOR_MAX_ACCELERATION 
			|| motorRotation.acceleration < CRD514KD::MOTOR_MIN_ACCELERATION
			|| motorRotation.deceleration > CRD514KD::MOTOR_MAX_ACCELERATION
			|| motorRotation.deceleration < CRD514KD::MOTOR_MIN_ACCELERATION){
			throw std::out_of_range("Acceleration or deceleration out of range.");
		}

		uint32_t motorSteps = (uint32_t)(motorRotation.angle / CRD514KD::MOTOR_STEP_ANGLE);
		if(useDeviation){
			motorSteps += (uint32_t)(deviation / CRD514KD::MOTOR_STEP_ANGLE);
		}

		uint32_t motorSpeed = (uint32_t)(motorRotation.speed / CRD514KD::MOTOR_STEP_ANGLE);

		// Formula to turn rad/s² into µs/kHz
		// 1000000 is for amount of microseconds in a second
		// 1000 is for amount of steps/s in a kHz
		uint32_t motorAcceleration = (uint32_t)((1000000/(motorRotation.acceleration/(CRD514KD::MOTOR_STEP_ANGLE * 1000))));
		uint32_t motorDeceleration = (uint32_t)((1000000/(motorRotation.deceleration/(CRD514KD::MOTOR_STEP_ANGLE * 1000))));

		// offset for the motion slot, * 2 for 32 bit registers.
		int motionSlotOffset = (motionSlot - 1) * 2;

		modbus->writeU32(motorIndex, CRD514KD::Registers::OP_SPEED + motionSlotOffset, motorSpeed, true);
		modbus->writeU32(motorIndex, CRD514KD::Registers::OP_POS + motionSlotOffset, motorSteps, true);
		modbus->writeU32(motorIndex, CRD514KD::Registers::OP_ACC + motionSlotOffset, motorAcceleration, true);
		modbus->writeU32(motorIndex, CRD514KD::Registers::OP_DEC + motionSlotOffset, motorDeceleration, true);
		setAngle = motorRotation.angle;
	}

	/**
	 * Start the motor to move according to the set registers. Will wait for the motor to be ready before moving.
	 **/
	void StepperMotor::startMovement(int motionSlot){
		checkMotionSlot(motionSlot);
		if(!poweredOn){
			throw MotorException("motor drivers are not powered on");
		}

		// Execute motion.
		waitTillReady();

		modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, motionSlot | CRD514KD::CMD1Bits::EXCITEMENT_ON | CRD514KD::CMD1Bits::START);
		modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
		updateAngle();
	}

	/**
	 * Wait till the motor indicates that the end location is reached.
	 **/
	void StepperMotor::waitTillReady(void){
		uint16_t status_1;
		while(!((status_1 = modbus->readU16(motorIndex, CRD514KD::Registers::STATUS_1)) & CRD514KD::Status1Bits::READY)){
			if((status_1 & CRD514KD::Status1Bits::ALARM) || (status_1 & CRD514KD::Status1Bits::WARNING)){
				std::cerr << "Motor: " << motorIndex << " Alarm code: " << std::hex << modbus->readU16(motorIndex, CRD514KD::Registers::PRESENT_ALARM) << "h" << std::endl;

				throw CRD514KDException(motorIndex, status_1 & CRD514KD::Status1Bits::WARNING, status_1 & CRD514KD::Status1Bits::ALARM);
			}
		}
	}

	/**
	 * Sets the deviation between the motors 0 degrees and the horizontal 0 degrees, then writes the new motor limits to the motor controllers.
	 *
	 * @param deviation The deviation between the hardware and theoretical 0 degrees.
	 **/
	void StepperMotor::setDeviationAndWriteMotorLimits(double deviation){
		this->deviation = deviation;
		modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((minAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
		modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((maxAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
	}

	/**
	 * Disables the limitations on the angles the motor can travel to in the motor hardware.
	 **/
	void StepperMotor::disableAngleLimitations(void){
		modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SOFTWARE_OVERTRAVEL, 0);
		anglesLimited = false;
	}

	/**
	 * Enables the limitations on the angles the motor can travel to in the motor hardware.
	 **/
	void StepperMotor::enableAngleLimitations(void){
		modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SOFTWARE_OVERTRAVEL, 1);
		anglesLimited = true;	
	}

	/**
	 * Store the angle that was given for a movement after the movement is done to the local variable currentAngle.
	 **/
	void StepperMotor::updateAngle(void){
		currentAngle = setAngle;
	}

	/**
	 * Sets the motor controller to incremental mode.
	 * @param motionSlot The motion slot to be set to incremental.
	 **/
	void StepperMotor::setIncrementalMode(int motionSlot){
		checkMotionSlot(motionSlot);
		modbus->writeU16(motorIndex, rexos_motor::CRD514KD::Registers::OP_POSMODE + motionSlot - 1, 0);
	}

	/**
	 * Sets the motor controller to absolute mode.
	 * @param motionSlot The motion slot to be set to absolute.
	 **/
	void StepperMotor::setAbsoluteMode(int motionSlot){
		checkMotionSlot(motionSlot);
		modbus->writeU16(motorIndex, rexos_motor::CRD514KD::Registers::OP_POSMODE + motionSlot - 1, 1);
	}

	/**
	 * Checks whether the motion slot is used. Throws an std::out_of_range exception if not.
	 * @param motionSlot the motion slot to be checked.
	 **/
	void StepperMotor::checkMotionSlot(int motionSlot){
		if(motionSlot < 1 || motionSlot > CRD514KD::MOTION_SLOTS_USED){
			std::cerr << "Motion slot: " << motionSlot << std::endl;
			throw std::out_of_range("Motion slot out of range.");
		}
	}
}
