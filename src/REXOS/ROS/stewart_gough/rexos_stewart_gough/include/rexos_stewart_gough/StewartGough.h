/**
 * @file StewartGough.h
 * @brief Symbolizes an entire StewartGough.
 *
 * @author Garik Hakopian
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

#include <modbus/modbus.h>
#include <rexos_stewart_gough/StewartGoughMeasures.h>
#include <rexos_stewart_gough/StewartGoughLocation.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_motor/MotorManager.h>
#include <rexos_motor/StepperMotorProperties.h>
#include <rexos_modbus/ModbusController.h>
#include <rexos_stewart_gough/SixAxisCalculations.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <jsoncpp/json/value.h>
#include "rexos_logger/rexos_logger.h"

#include <vector>

namespace rexos_stewart_gough{
	struct MotorMap {
		int motor;
		int sensor;
		
		MotorMap(int motor = 0, int sensor = 0):
				motor(motor),
				sensor(sensor){}
	};
	
	struct MotorGroup {
		int motorIndex1;
		int motorIndex2;
		
		MotorGroup(int motorIndex1 = 0, int motorIndex2 = 0):
				motorIndex1(motorIndex1),
				motorIndex2(motorIndex2){}
	};
	
	/**
	 * A class that symbolizes an entire deltarobot.
	 **/
	class StewartGough{
	public:
		StewartGough(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, Json::Value node);
		~StewartGough();
		
		void readJSONNode(Json::Value node);
		
		bool checkPath(const StewartGoughLocation& begin, const StewartGoughLocation& end);
		void moveTo(StewartGoughLocation point, double maxAcceleration);
		
		void calibrateMotor(int motorIndex1,int motorIndex2);
		bool checkSensor(int sensorIndex);
		bool calibrateMotors();
		void powerOff();
		void powerOn();
		
		StewartGoughLocation getEffectorLocation();
		
	private:
		std::string equipletName;
		rexos_datatypes::ModuleIdentifier identifier;
		bool isSimulated;
		
		/**
		 * @var StepperMotor* motors
		 * An array holding pointers to the three StepperMotors that are connected to the DeltaRobot. This array HAS to be of size 3.
		 **/
		std::vector<rexos_motor::MotorInterface*> motors;
		
		/**
		 *
		 * @var ModbusController::ModbusController* modbus
		 * the modbuscontroller
		 **/
		rexos_modbus::ModbusController* modbus;
		void createModbus();
		
		// the inital motors and sensors are positioned on the wrong locations
		// this is used to map them on the right locations
		MotorMap motorMap[6];
		
		//SixAxisCalculations * sixAxisCalculations;
		
		rexos_stewart_gough::StewartGoughMeasures stewartGoughMeasures;
		rexos_motor::StepperMotorProperties stepperMotorProperties;
		
		/**
		 * @var MotorManager* motorManager
		 * A pointer to the MotorManager that handles the movement for the DeltaRobot.
		 **/
		rexos_motor::MotorManager* motorManager;
		void createMotorManager();
		
		/**
		 * @var modbus_t* modbusIO
		 * A pointer to the TCP modbus connection for the IO controller.
		 **/
		modbus_t* modbusIO;
		
		SixAxisCalculations* sixAxisCalculations;
		
		
		/**
		 * @var Point3D<double> effectorLocation
		 * A 3D point in doubles that points to the location of the effector.
		 **/
		StewartGoughLocation effectorLocation;
		

		int calibrationBigStepFactor;
		std::string modbusIp;
		int modbusPort;
		
		
		/**
		 * @var int currentMotionSlot
		 * The motion slot currently in use. The deltarobot switches between these slots when moving.
		 **/
		int currentMotionSlot;
		
		bool isValidAngle(int motorIndex, double angle);
		MotorGroup moveMotorUntilSensorIsOfValue(int motorIndex1,int motorIndex2, rexos_motor::MotorRotation motorRotation1 ,rexos_motor::MotorRotation motorRotation2, bool sensorValue);
		double getSpeedForRotation(double relativeAngle, double moveTime, double acceleration);
		double getAccelerationForRotation(double relativeAngle, double moveTime);
		
		rexos_motor::MotorInterface* getMotor(int number);
		int getMotorIndexByNumber(int number);
		
		void deleteMotorRotationObjects(rexos_motor::MotorRotation* rotations[6]);
	};
}
