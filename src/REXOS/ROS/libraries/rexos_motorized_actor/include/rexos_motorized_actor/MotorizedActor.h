/**
 * @file DeltaRobot.h
 * @brief Symbolizes an entire deltarobot.
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

#include <jsoncpp/json/value.h>

#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_motor/MotorManager.h>
#include <rexos_motor/StepperMotorProperties.h>
#include <rexos_sensor/ContactSensor.h>
#include <rexos_io/InputOutputControllerInterface.h>
#include <rexos_io/RtuModbusInputOutputController.h>
#include <rexos_logger/rexos_logger.h>


namespace rexos_motorized_actor{
	class MotorizedActor{
	protected:
		MotorizedActor(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, int numberOfMotors, Json::Value node);
		~MotorizedActor();
		
		void calibrateMotorGroup(std::vector<rexos_motor::MotorInterface*> motors, 
			std::vector<rexos_sensor::ContactSensor*> sensor);
	public:
		virtual bool calibrateMotors() = 0;
		/**
		 * Shuts down the deltarobot's hardware.
		**/
		void powerOff();
		/**
		 * Turns on the deltarobot's hardware.
		**/
		void powerOn();
	protected:
		std::string equipletName;
		rexos_datatypes::ModuleIdentifier identifier;
		bool isSimulated;
		
		rexos_motor::StepperMotorProperties stepperMotorProperties;
		int calibrationBigStepFactor;
		/**
		 * @var double contactSensorToZeroAngle
		 * The angle from zero (horizontal) to the top position in radians.
		 **/
		double contactSensorToZeroAngle;
		
		/**
		 * @var StepperMotor* motors
		 * An array holding pointers to the three StepperMotors that are connected to the DeltaRobot. This array HAS to be of size 3.
		 **/
		std::vector<rexos_motor::MotorInterface*> motors;
		std::vector<rexos_sensor::ContactSensor*> sensors;
		
		/**
		 * @var MotorManager* motorManager
		 * A pointer to the MotorManager that handles the movement for the DeltaRobot.
		 **/
		rexos_motor::MotorManager* motorManager;


	private:
		/**
		 * @var modbus_t* modbusIO
		 * A pointer to the TCP modbus connection for the IO controller.
		 **/
		rexos_io::InputOutputControllerInterface* sensorIOController;
		/**
		 *
		 * @var ModbusController::ModbusController* modbus
		 * the modbuscontroller
		 **/
		rexos_io::RtuModbusInputOutputController* motorIOController;
		
		void readJSONNode(Json::Value node);
		bool checkSensors();
		/**
		 * Incrementally moves the motor until the sensor is of the value given in sensorValue. 
		 * Calculates how many steps are made, and returns this as an int. 
		 *
		 * @param motorIndex The index of the motor
		 * @param motorRotation The MotorRotation object that holds that the rotation data.
		 * @param sensorValue value that the sensor needs to be for the motor to stop.
		 *
		 * @return The amount of motor steps the motor has moved.
		 **/
		std::vector<double> rotateMotorGroupUntillSensorIfOfValue(std::vector<rexos_motor::MotorInterface*> motors, 
			std::vector<rexos_sensor::ContactSensor*> sensor, rexos_motor::MotorRotation motorRotation, bool desiredSensorValue);
	};
}
