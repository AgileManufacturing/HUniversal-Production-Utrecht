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
#include <limits>

#include <rexos_utilities/Utilities.h>
#include <rexos_motor/CRD514KD.h>
#include <rexos_motor/CRD514KDException.h>
#include <rexos_motor/MotorException.h>
#include <motor_manager_plugin/isMotorReady.h>
#include <motor_manager_plugin/setMinSpeed.h>
#include <motor_manager_plugin/setLowerAngleLimit.h>
#include <motor_manager_plugin/setUpperAngleLimit.h>
#include <motor_manager_plugin/setMotorMode.h>
#include <motor_manager_plugin/setPowerStatus.h>
#include <motor_manager_plugin/startMotor.h>
#include <motor_manager_plugin/stopMotor.h>
#include <motor_manager_plugin/writeRotationData.h>

namespace rexos_motor {
	SimulatedMotor::SimulatedMotor(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, int index, MotorProperties properties) :
			MotorInterface(properties), equipletName(equipletName), identifier(identifier), index(index), nodeHandle() {
		std::string servicePath = equipletName + "/" + identifier.getManufacturer() + "/" + 
				identifier.getTypeNumber() + "/" + identifier.getSerialNumber() + "/";
		
		isMotorReadyClient = 		nodeHandle.serviceClient<motor_manager_plugin::isMotorReady>(servicePath + "isMotorReady");
		setMinSpeedClient = 		nodeHandle.serviceClient<motor_manager_plugin::setMinSpeed>(servicePath + "setMinSpeed");
		setLowerAngleLimitClient = nodeHandle.serviceClient<motor_manager_plugin::setLowerAngleLimit>(servicePath + "setLowerAngleLimit");
		setUpperAngleLimitClient = nodeHandle.serviceClient<motor_manager_plugin::setUpperAngleLimit>(servicePath + "setUpperAngleLimit");
		setMotorModeClient = 		nodeHandle.serviceClient<motor_manager_plugin::setMotorMode>(servicePath + "setMotorMode");
		setPowerStatusClient = 		nodeHandle.serviceClient<motor_manager_plugin::setPowerStatus>(servicePath + "setPowerStatus");
		startMotorClient = 			nodeHandle.serviceClient<motor_manager_plugin::startMotor>(servicePath + "startMotor");
		stopMotorClient = 			nodeHandle.serviceClient<motor_manager_plugin::stopMotor>(servicePath + "stopMotor");
		writeRotationDataClient = 	nodeHandle.serviceClient<motor_manager_plugin::writeRotationData>(servicePath + "writeRotationData");
	}

	SimulatedMotor::~SimulatedMotor(void) {
		powerOff();
	}

	void SimulatedMotor::powerOn(void) {
		if(!isPoweredOn()){
			motor_manager_plugin::setLowerAngleLimit lowerAngleLimitCall;
			lowerAngleLimitCall.request.motorIndex = index;
			lowerAngleLimitCall.request.angle = deviation + properties.motorMinAngle;
			setLowerAngleLimitClient.waitForExistence();
			setLowerAngleLimitClient.call(lowerAngleLimitCall);
			
			motor_manager_plugin::setUpperAngleLimit upperAngleLimitCall;
			upperAngleLimitCall.request.motorIndex = index;
			upperAngleLimitCall.request.angle = deviation + properties.motorMaxAngle;
			setUpperAngleLimitClient.waitForExistence();
			setUpperAngleLimitClient.call(upperAngleLimitCall);
			
			motor_manager_plugin::setMinSpeed minSpeedCall;
			minSpeedCall.request.motorIndex = index;
			minSpeedCall.request.velocity = properties.minSpeed;
			setMinSpeedClient.waitForExistence();
			setMinSpeedClient.call(minSpeedCall);
			
			motor_manager_plugin::setPowerStatus powerStatusCall;
			powerStatusCall.request.motorIndex = index;
			powerStatusCall.request.powerStatus = true;
			setPowerStatusClient.waitForExistence();
			setPowerStatusClient.call(powerStatusCall);
			
			currentAngle = 0;
			powerStatus = true;
		}
	}

	void SimulatedMotor::powerOff(void) {
		if(isPoweredOn()){
			stop();
			
			motor_manager_plugin::setPowerStatus powerStatusCall;
			powerStatusCall.request.motorIndex = index;
			powerStatusCall.request.powerStatus = false;
			setPowerStatusClient.waitForExistence();
			setPowerStatusClient.call(powerStatusCall);
			
			powerStatus = false;
		}
	}

	void SimulatedMotor::stop(void) {
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}
		
		motor_manager_plugin::stopMotor stopMotorCall;
		stopMotorCall.request.motorIndex = index;
		stopMotorClient.waitForExistence();
		stopMotorClient.call(stopMotorCall);
	}

	void SimulatedMotor::resetCounter(void) {
		waitTillReady();
		targetAngle = 0;
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
			throw std::out_of_range("Deacceleration out of range. Got " + boost::lexical_cast<std::string>(motorRotation.deceleration) + 
					", expecting between " + boost::lexical_cast<std::string>(properties.minAcceleration)+ "  and " + 
					boost::lexical_cast<std::string>(properties.maxAcceleration));
		}
		
		double motorAngle = motorRotation.angle;
		if(useDeviation){
			motorAngle += deviation;
		}
		
		// we cannot write rotation data while motor is running
		waitTillReady();
		
		motor_manager_plugin::writeRotationData writeRotationDataCall;
		writeRotationDataCall.request.motorIndex = index;
		writeRotationDataCall.request.maxAcceleration = motorRotation.acceleration;
		writeRotationDataCall.request.maxDecelleration = motorRotation.deceleration;
		writeRotationDataCall.request.maxSpeed = motorRotation.speed;
		writeRotationDataCall.request.angle = motorAngle;
		writeRotationDataClient.waitForExistence();
		writeRotationDataClient.call(writeRotationDataCall);
		
		targetAngle = motorRotation.angle;
	}

	void SimulatedMotor::startMovement() {
		if(!isPoweredOn()){
			throw MotorException("motor drivers are not powered on");
		}
		// wait for motor to be ready before starting new movement
		waitTillReady();
		
		motor_manager_plugin::startMotor startMotorCall;
		startMotorCall.request.motorIndex = index;
		startMotorClient.waitForExistence();
		startMotorClient.call(startMotorCall);

		updateAngle();
	}

	void SimulatedMotor::waitTillReady(void) {
		// TODO perhaps do something better? Use mutexes?
		while(isReady() == false) {
			// no need to wait as the isReady call jams itself waiting for response
		}
	}
	
	bool SimulatedMotor::isReady(void) {
		motor_manager_plugin::isMotorReady isMotorReadyCall;
		isMotorReadyCall.request.motorIndex = index;
		isMotorReadyClient.waitForExistence();
		if(isMotorReadyClient.call(isMotorReadyCall) == false) {
			throw std::runtime_error("Error calling service");
		}
		return isMotorReadyCall.response.isMotorReady;
	}
	
	void SimulatedMotor::setRelativeMode(void) {
		motor_manager_plugin::setMotorMode setMotorModeCall;
		setMotorModeCall.request.motorIndex = index;
		setMotorModeCall.request.relativeMode = true;
		setMotorModeClient.waitForExistence();
		setMotorModeClient.call(setMotorModeCall);
	}

	void SimulatedMotor::setAbsoluteMode(void) {
		motor_manager_plugin::setMotorMode setMotorModeCall;
		setMotorModeCall.request.motorIndex = index;
		setMotorModeCall.request.relativeMode = false;
		setMotorModeClient.waitForExistence();
		setMotorModeClient.call(setMotorModeCall);
	}
	void SimulatedMotor::setDeviation(double deviationAngle) {
		deviation = deviationAngle;
		if(anglesLimited == true) {
			motor_manager_plugin::setLowerAngleLimit lowerAngleLimitCall;
			lowerAngleLimitCall.request.motorIndex = index;
			lowerAngleLimitCall.request.angle = deviation + properties.motorMinAngle;
			setLowerAngleLimitClient.waitForExistence();
			setLowerAngleLimitClient.call(lowerAngleLimitCall);
			
			motor_manager_plugin::setUpperAngleLimit upperAngleLimitCall;
			upperAngleLimitCall.request.motorIndex = index;
			upperAngleLimitCall.request.angle = deviation + properties.motorMaxAngle;
			setUpperAngleLimitClient.waitForExistence();
			setUpperAngleLimitClient.call(upperAngleLimitCall);
		}
	}
	
	void SimulatedMotor::enableAngleLimitations() {
		anglesLimited = true;
		
		motor_manager_plugin::setLowerAngleLimit lowerAngleLimitCall;
		lowerAngleLimitCall.request.motorIndex = index;
		lowerAngleLimitCall.request.angle = deviation + properties.motorMinAngle;
		setLowerAngleLimitClient.waitForExistence();
		setLowerAngleLimitClient.call(lowerAngleLimitCall);
		
		motor_manager_plugin::setUpperAngleLimit upperAngleLimitCall;
		upperAngleLimitCall.request.motorIndex = index;
		upperAngleLimitCall.request.angle = deviation + properties.motorMaxAngle;
		setUpperAngleLimitClient.waitForExistence();
		setUpperAngleLimitClient.call(upperAngleLimitCall);
	}
	
	void SimulatedMotor::disableAngleLimitations() {
		anglesLimited = false;
		
		// setting the limits to infinity has the same effect as disabling them
		motor_manager_plugin::setLowerAngleLimit lowerAngleLimitCall;
		lowerAngleLimitCall.request.motorIndex = index;
		lowerAngleLimitCall.request.angle = -std::numeric_limits<double>::infinity();
		setLowerAngleLimitClient.waitForExistence();
		setLowerAngleLimitClient.call(lowerAngleLimitCall);
		
		motor_manager_plugin::setUpperAngleLimit upperAngleLimitCall;
		upperAngleLimitCall.request.motorIndex = index;
		upperAngleLimitCall.request.angle = std::numeric_limits<double>::infinity();
		setUpperAngleLimitClient.waitForExistence();
		setUpperAngleLimitClient.call(upperAngleLimitCall);
	}
}
