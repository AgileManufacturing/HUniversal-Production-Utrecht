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

#include <rexos_motor/SimulatedMotor.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <rexos_utilities/Utilities.h>
#include <rexos_motor/CRD514KD.h>
#include <rexos_motor/CRD514KDException.h>
#include <rexos_motor/MotorException.h>

namespace rexos_motor {
	SimulatedMotor::SimulatedMotor(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, int index, MotorProperties properties):
			MotorInterface(properties), equipletName(equipletName), identifier(identifier), index(index) {
	}

	SimulatedMotor::~SimulatedMotor(void) {
		powerOff();
	}

	void SimulatedMotor::powerOn(void) {
		if(!isPoweredOn()){
			//
			
			currentAngle = 0;
			powerStatus = true;
		}
	}

	void SimulatedMotor::powerOff(void) {
		if(isPoweredOn()){
			stop();
			
			//
			
			powerStatus = false;
		}
	}

	void SimulatedMotor::stop(void) {
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}
		//
	}

	void SimulatedMotor::resetCounter(void) {
		waitTillReady();
		//
	}

	void SimulatedMotor::writeRotationData(const rexos_motor::MotorRotation& motorRotation, bool useDeviation) {
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}

		if(anglesLimited && isValidAngle(motorRotation.angle) == false) {
			throw std::out_of_range("one or more angles out of range");
		}

		if(motorRotation.acceleration > properties.maxAcceleration || motorRotation.acceleration < properties.minAcceleration) {
			throw std::out_of_range("Acceleration out of range.");
		}
		if(motorRotation.deceleration > properties.maxAcceleration || motorRotation.deceleration < properties.minAcceleration) {
			throw std::out_of_range("Deacceleration out of range.");
		}
		
		uint32_t motorAngle = (uint32_t) motorRotation.angle;
		if(useDeviation){
			motorAngle += (uint32_t) deviation;
		}

		uint32_t motorSpeed = (uint32_t) motorRotation.speed;

		// Formula to turn rad/s² into µs/kHz
		// 1000000 is for amount of microseconds in a second
		// 1000 is for amount of steps/s in a kHz
		uint32_t motorAcceleration = (uint32_t) motorRotation.acceleration;
		uint32_t motorDeceleration = (uint32_t) motorRotation.deceleration;
		
		//

		targetAngle = motorRotation.angle;
	}

	void SimulatedMotor::startMovement() {
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}
		// wait for motor to be ready before starting new movement
		waitTillReady();
		
		//
		updateAngle();
	}

	void SimulatedMotor::waitTillReady(void) {
		//
	}
	
	bool SimulatedMotor::isReady(void) {
		//
	}
	
	void SimulatedMotor::setDeviationAndWriteMotorLimits(double deviation) {
		this->deviation = deviation;
		//
	}

	void SimulatedMotor::setRelativeMode(void) {
		//
	}

	void SimulatedMotor::setAbsoluteMode(void) {
		//
	}
	void SimulatedMotor::setDeviation(double deviationAngle) {
		deviation = deviationAngle;
		
		//
	}
	
	void SimulatedMotor::enableAngleLimitations() {
		anglesLimited = true;
		
		//
	}
	
	void SimulatedMotor::disableAngleLimitations() {
		anglesLimited = false;
		
		//
	}
}
