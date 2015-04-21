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
	StepperMotor::StepperMotor(rexos_io::RtuModbusInputOutputController* ioController, uint16_t motorAddress, StepperMotorProperties properties):
			MotorInterface(properties), properties(properties), ioController(ioController), motorAddress(motorAddress) {
	}

	StepperMotor::~StepperMotor(void){
		powerOff();
	}

	void StepperMotor::powerOn(void) {
		if(!isPoweredOn()){
			//Reset alarm
			ioController->writeU16(motorAddress, CRD514KD::Registers::RESET_ALARM, 0, false);
			ioController->writeU16(motorAddress, CRD514KD::Registers::RESET_ALARM, 1, false);
			ioController->writeU16(motorAddress, CRD514KD::Registers::RESET_ALARM, 0, false);
			
			// Set operating modes
			ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, 0, false);
			
			// set modes for all used motion slots
			for(int i = 0; i < CRD514KD::MOTION_SLOTS_USED; i++){
				ioController->writeU16(motorAddress, CRD514KD::Registers::OP_POSMODE + i, 1, false);
				ioController->writeU16(motorAddress, CRD514KD::Registers::OP_OPMODE + i, 0, false);
				ioController->writeU16(motorAddress, CRD514KD::Registers::OP_SEQ_MODE + i, 1, false);
			}
			// Excite motor
			ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON, false);
			
			// Set motor limits
			ioController->writeU32(motorAddress, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, 
					(uint32_t)((getMaxAngle() - deviation) / properties.microStepAngle), false);
			ioController->writeU32(motorAddress, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, 
					(uint32_t)((getMinAngle() - deviation) / properties.microStepAngle), false);
			ioController->writeU32(motorAddress, CRD514KD::Registers::CFG_START_SPEED, 1, false);
			
			// Clear counter
			ioController->writeU16(motorAddress, CRD514KD::Registers::CLEAR_COUNTER, 1, false);
			ioController->writeU16(motorAddress, CRD514KD::Registers::CLEAR_COUNTER, 0, false);

			currentAngle = 0;
			powerStatus = true;
		}
	}

	void StepperMotor::powerOff(void){
		if(isPoweredOn()){
			stop();
			ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, 0, false);
			powerStatus = false;
		}
	}

	void StepperMotor::stop(void){
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}
		ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::STOP, false);
		ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON, false);
	}

	void StepperMotor::resetCounter(void){
		waitTillReady();
		ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, 0, false);

		// Clear counter.
		ioController->writeU16(motorAddress, CRD514KD::Registers::CLEAR_COUNTER, 1, false);
		ioController->writeU16(motorAddress, CRD514KD::Registers::CLEAR_COUNTER, 0, false);
		ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON, false);
	}

	void StepperMotor::moveTo(const rexos_motor::MotorRotation& motorRotation, int motionSlot){
		checkMotionSlot(motionSlot);

		writeRotationData(motorRotation, motionSlot);
		startMovement(motionSlot);
	}


	void StepperMotor::writeRotationData(const rexos_motor::MotorRotation& motorRotation, bool useDeviation) {
		writeRotationData(motorRotation, DEFAULT_MOTION_SLOT, useDeviation);
	}
	void StepperMotor::writeRotationData(const rexos_motor::MotorRotation& motorRotation, int motionSlot, bool useDeviation) {
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}

		checkMotionSlot(motionSlot);

		if(anglesLimited && isValidAngle(motorRotation.angle) == false) {
			throw std::out_of_range("one or more angles out of range");
		}

		if(motorRotation.acceleration > properties.maxAcceleration || motorRotation.acceleration < properties.minAcceleration) {
			throw std::out_of_range("Acceleration out of range.");
		}
		if(motorRotation.deceleration > properties.maxAcceleration || motorRotation.deceleration < properties.minAcceleration) {
			throw std::out_of_range("Deacceleration out of range.");
		}
		
		uint32_t motorSteps = (uint32_t)(motorRotation.angle / properties.microStepAngle);
		if(useDeviation){
			motorSteps += (uint32_t)(deviation / properties.microStepAngle);
		}

		uint32_t motorSpeed = (uint32_t)(motorRotation.speed / properties.microStepAngle);

		// Formula to turn rad/s² into µs/kHz
		// 1000000 is for amount of microseconds in a second
		// 1000 is for amount of steps/s in a kHz
		uint32_t motorAcceleration = (uint32_t)((1000000/(motorRotation.acceleration/(properties.microStepAngle * 1000))));
		uint32_t motorDeceleration = (uint32_t)((1000000/(motorRotation.deceleration/(properties.microStepAngle * 1000))));

		// offset for the motion slot, * 2 for 32 bit registers.
		uint16_t motionSlotOffset = (motionSlot - 1) * 2;

		ioController->writeU32(motorAddress, CRD514KD::Registers::OP_SPEED + motionSlotOffset, motorSpeed, true);
		ioController->writeU32(motorAddress, CRD514KD::Registers::OP_POS + motionSlotOffset, motorSteps, true);
		ioController->writeU32(motorAddress, CRD514KD::Registers::OP_ACC + motionSlotOffset, motorAcceleration, true);
		ioController->writeU32(motorAddress, CRD514KD::Registers::OP_DEC + motionSlotOffset, motorDeceleration, true);
		targetAngle = motorRotation.angle;
	}

	void StepperMotor::startMovement() {
		startMovement(DEFAULT_MOTION_SLOT);
	}
	void StepperMotor::startMovement(int motionSlot) {
		checkMotionSlot(motionSlot);
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}
		// wait for motor to be ready before starting new movement
		waitTillReady();
		
		ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, motionSlot | CRD514KD::CMD1Bits::EXCITEMENT_ON | CRD514KD::CMD1Bits::START, false);
		ioController->writeU16(motorAddress, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON, false);
		updateAngle();
	}

	void StepperMotor::waitTillReady(void){
		uint16_t status_1;
		while(!((status_1 = ioController->readU16(motorAddress, CRD514KD::Registers::STATUS_1, false)) & CRD514KD::Status1Bits::READY)) {
			if((status_1 & CRD514KD::Status1Bits::ALARM) || (status_1 & CRD514KD::Status1Bits::WARNING)) {
				throw CRD514KDException(motorAddress, status_1 & CRD514KD::Status1Bits::WARNING, status_1 & CRD514KD::Status1Bits::ALARM);
			}
		}
	}
	
	bool StepperMotor::isReady(void){
		uint16_t status_1;
		if(!((status_1 = ioController->readU16(motorAddress, CRD514KD::Registers::STATUS_1, false)) & CRD514KD::Status1Bits::READY)){
			if((status_1 & CRD514KD::Status1Bits::ALARM) || (status_1 & CRD514KD::Status1Bits::WARNING)){

				throw CRD514KDException(motorAddress, status_1 & CRD514KD::Status1Bits::WARNING, status_1 & CRD514KD::Status1Bits::ALARM);
			}
			return false;
		}
		return true;
	}
	void StepperMotor::setRelativeMode(){
		for(int motionSlot = 0; motionSlot < CRD514KD::MOTION_SLOTS_USED; motionSlot++) {
			ioController->writeU16(motorAddress, rexos_motor::CRD514KD::Registers::OP_POSMODE + motionSlot - 1, 0, false);
		}
	}

	void StepperMotor::setAbsoluteMode(){
		for(int motionSlot = 0; motionSlot < CRD514KD::MOTION_SLOTS_USED; motionSlot++) {
			ioController->writeU16(motorAddress, rexos_motor::CRD514KD::Registers::OP_POSMODE + motionSlot - 1, 1, false);
		}
	}

	void StepperMotor::checkMotionSlot(int motionSlot){
		if(motionSlot < 1 || motionSlot > CRD514KD::MOTION_SLOTS_USED){
			throw std::out_of_range("Motion slot out of range.");
		}
	}
	
	void StepperMotor::setDeviation(double deviationAngle) {
		deviation = deviationAngle;
		ioController->writeU32(motorAddress, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, 
				(uint32_t)((getMinAngle() + deviation) / properties.microStepAngle), false);
		ioController->writeU32(motorAddress, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, 
				(uint32_t)((getMaxAngle() + deviation) / properties.microStepAngle), false);
	}
	
	void StepperMotor::enableAngleLimitations() {
		ioController->writeU16(motorAddress, CRD514KD::Registers::OP_SOFTWARE_OVERTRAVEL, 1, false);
		anglesLimited = true;
	}
	
	void StepperMotor::disableAngleLimitations() {
		ioController->writeU16(motorAddress, CRD514KD::Registers::OP_SOFTWARE_OVERTRAVEL, 0, false);
		anglesLimited = false;
	}
}
