/**
 * @file DeltaRobot.cpp
 * @brief Symbolizes an entire deltarobot.
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Koen Braham
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

#include <string>
#include <stdexcept>

#include <rexos_motorized_actor/MotorizedActor.h>
#include <rexos_motor/MotorException.h>
#include <rexos_motor/SimulatedMotor.h>
#include <rexos_motor/StepperMotor.h>
#include <rexos_motor/SimulatedMotorManager.h>
#include <rexos_motor/StepperMotorManager.h>
#include <rexos_sensor/SimulatedContactSensor.h>
#include <rexos_sensor/ContactSensor.h>
#include <rexos_utilities/Utilities.h>

#include "ros/ros.h"

namespace rexos_motorized_actor{
	MotorizedActor::MotorizedActor(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, 
			int numberOfMotors, Json::Value node) :
			equipletName(equipletName), identifier(moduleIdentifier), isSimulated(isSimulated), 
			stepperMotorProperties(node["stepperMotorProperties"]), 
			motorManager(NULL),
			modbusIO(NULL), modbus(NULL) {
		readJSONNode(node);
		
		REXOS_INFO("Configuring sensors");
		if(isSimulated == true) {
			for(int i = 0; i < numberOfMotors; i++) {
				sensors.push_back(new rexos_sensor::SimulatedContactSensor(equipletName, identifier, i));
			}
		} else {
			// Initialize modbus for IO controller
			modbusIO = modbus_new_tcp(modbusIp.c_str(), modbusPort);
			if(modbusIO == NULL){
				throw std::runtime_error("Unable to allocate libmodbus context");
			}
			if(modbus_connect(modbusIO) == -1) {
				throw std::runtime_error("Modbus connection to IO controller failed");
			}
			
			for(int i = 0; i < numberOfMotors; i++) {
				sensors.push_back(new rexos_sensor::ContactSensor(i, modbusIO));
			}
		}
		
		// initialize motors
		if(isSimulated == true) {
			for(int i = 0; i < numberOfMotors; i++) {
				motors.push_back(new rexos_motor::SimulatedMotor(equipletName, identifier, i, stepperMotorProperties));
			}
			motorManager = new rexos_motor::SimulatedMotorManager(equipletName, identifier, motors);
		} else {
			modbus = new rexos_modbus::ModbusController(modbus_new_rtu(
				"/dev/ttyS0",
				rexos_motor::CRD514KD::RtuConfig::BAUDRATE,
				rexos_motor::CRD514KD::RtuConfig::PARITY,
				rexos_motor::CRD514KD::RtuConfig::DATA_BITS,
				rexos_motor::CRD514KD::RtuConfig::STOP_BITS));
			
			for(int i = 0; i < numberOfMotors; i++) {
				motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::SlaveAddresses::MOTOR_0 + i, stepperMotorProperties));
			}
			motorManager = new rexos_motor::StepperMotorManager(modbus, motors);
		}
	}

	/**
	 * Deconstructor of a deltarobot. Turns off the motors and deletes the kinematics model.
	 **/
	MotorizedActor::~MotorizedActor(void){
		if(motorManager->isPoweredOn()){
			motorManager->powerOff();
		}
	}
	
	void MotorizedActor::powerOff(void){
		if(motorManager->isPoweredOn()){
			motorManager->powerOff();
		}
	}

	void MotorizedActor::powerOn(void){
		if(!motorManager->isPoweredOn()){
			motorManager->powerOn();
		}
	}
	void MotorizedActor::readJSONNode(const Json::Value node) {
		modbusIp = node["modbusIp"].asString();
		REXOS_INFO_STREAM("found modbusIp " << modbusIp);
		
		modbusPort = node["modbusPort"].asInt();
		REXOS_INFO_STREAM("found modbusPort " << modbusPort);
		
		calibrationBigStepFactor = node["calibrationBigStepFactor"].asInt();
		REXOS_INFO_STREAM("found calibrationBigStepFactor " << calibrationBigStepFactor);
		
		contactSensorToZeroAngle = rexos_utilities::degreesToRadians(node["contactSensorToZeroAngleDegrees"].asDouble());
		REXOS_INFO_STREAM("found contactSensorToZeroAngleDegrees " << contactSensorToZeroAngle);
		
		
		Json::Value stepperMotorPropertiesNode = node["stepperMotorProperties"];
		stepperMotorProperties = rexos_motor::StepperMotorProperties(stepperMotorPropertiesNode);
		REXOS_INFO_STREAM("found stepperMotorProperties");
	}
	
	bool MotorizedActor::checkSensors() {
		// Check the availability of the sensors
		for(uint i = 0; i < sensors.size(); i++) {
			if(sensors[i]->isTriggered() == true) {
				REXOS_ERROR_STREAM("Sensor " << i << " failure (is the hardware connected?)");
				return false;
			}
		}
		return true;
	}
	
	std::vector<double> MotorizedActor::rotateMotorGroupUntillSensorIfOfValue(std::vector<rexos_motor::MotorInterface*> motorsToCalibrate, 
			std::vector<rexos_sensor::ContactSensorInterface*> sensorsToUse, rexos_motor::MotorRotation motorRotation, bool desiredSensorValue) {
		std::vector<bool> hasDesiredValue;
		std::vector<double> deviations;
		for(uint i = 0; i < motorsToCalibrate.size(); i++) {
			motorsToCalibrate[i]->writeRotationData(motorRotation, false);
			hasDesiredValue.push_back(false);
			deviations.push_back(0);
		}
		
		// as long as one sensor has not desired value
		while(std::find(hasDesiredValue.begin(), hasDesiredValue.end(), false) != hasDesiredValue.end()) {
			for(uint i = 0; i < motorsToCalibrate.size(); i++) {
				if(hasDesiredValue[i] == false) {
					motorsToCalibrate[i]->startMovement();
					motorsToCalibrate[i]->waitTillReady();
					deviations[i] += motorRotation.angle;
					if(sensorsToUse[i]->isTriggered() == desiredSensorValue) {
						REXOS_INFO_STREAM("sensor " << i << " of group has desired value");
						hasDesiredValue[i] = true;
					}
				}
			}
		}
		return deviations;
	}
	void MotorizedActor::calibrateMotorGroup(std::vector<rexos_motor::MotorInterface*> motorsToCalibrate, 
			std::vector<rexos_sensor::ContactSensorInterface*> sensorsToUse) {
		if(motorsToCalibrate.size() != sensorsToUse.size()) {
			throw std::runtime_error("motorsToCalibrate.size() != sensorsToUse.size()");
		}
		
		rexos_motor::MotorRotation motorRotation;
		std::vector<double> deviations;
		for(uint i = 0; i < motorsToCalibrate.size(); i++) {
			deviations.push_back(0);
			motorsToCalibrate[i]->disableAngleLimitations();
			motorsToCalibrate[i]->setRelativeMode();
		}
		
		REXOS_INFO("fase 1 starting");
		// Move to the sensor in big steps until it is pushed
		motorRotation.angle = 0 - stepperMotorProperties.microStepAngle * calibrationBigStepFactor;
		std::vector<double> bigStepDeviation = rotateMotorGroupUntillSensorIfOfValue(motorsToCalibrate, sensorsToUse, motorRotation, true);
		for(uint i = 0; i < deviations.size(); i++) {
			deviations[i] += bigStepDeviation[i];
		}
		REXOS_INFO("fase 1 done");
		// Move away from the sensor in big steps until it is no longer pushed.
		motorRotation.angle = stepperMotorProperties.microStepAngle * calibrationBigStepFactor;
		std::vector<double> backOffDeviation = rotateMotorGroupUntillSensorIfOfValue(motorsToCalibrate, sensorsToUse, motorRotation, false);
		for(uint i = 0; i < deviations.size(); i++) {
			// we can add as the deviation return from the fuction will be negative
			deviations[i] += backOffDeviation[i];
		}
		REXOS_INFO("fase 2 done");
		// Move back to the sensor in small steps until it is pushed.
		motorRotation.angle = 0 - stepperMotorProperties.microStepAngle;
		std::vector<double> smallStepDeviation = rotateMotorGroupUntillSensorIfOfValue(motorsToCalibrate, sensorsToUse, motorRotation, true);
		for(uint i = 0; i < deviations.size(); i++) {
			deviations[i] += smallStepDeviation[i];
		}
		REXOS_INFO("fase 3 done");
		
		
		// store the measured deviation
		for(uint i = 0; i < motorsToCalibrate.size(); i++) {
			deviations[i] += contactSensorToZeroAngle;
			motorsToCalibrate[i]->setDeviation(deviations[i]);
			motorsToCalibrate[i]->enableAngleLimitations();
			motorsToCalibrate[i]->setAbsoluteMode();
		}
		// rotate the motors back to 0
		motorRotation.angle = 0;
		for(uint i = 0; i < motorsToCalibrate.size(); i++) {
			motorsToCalibrate[i]->moveTo(motorRotation);
		}
		for(uint i = 0; i < motorsToCalibrate.size(); i++) {
			motorsToCalibrate[i]->waitTillReady();
		}
	}
}
