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

#include <modbus/modbus.h>
#include <vectors/Vectors.h>
#include <rexos_delta_robot/DeltaRobotMeasures.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_motor/StepperMotor.h>
#include <rexos_motor/MotorManager.h>
#include <rexos_motor/StepperMotorProperties.h>
#include <rexos_delta_robot/DeltaRobotMeasures.h>
#include <rexos_delta_robot/EffectorBoundaries.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <jsoncpp/json/value.h>
#include "rexos_logger/rexos_logger.h"

#include <vector>

namespace rexos_delta_robot{
	class InverseKinematicsModel;
	/**
	 * A class that symbolizes an entire deltarobot.
	 **/
	class DeltaRobot{
	public:
		DeltaRobot(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, Json::Value node);
		~DeltaRobot();
		
		void readJSONNode(Json::Value node);

		/**
		 * Gets the EffectorBoundaries of the deltarobot.
		 * @return The EffectorBoundaries of the deltarobot.
		 **/
		inline EffectorBoundaries* getBoundaries(){ return boundaries; }

		/**
		 * Checks whether the EffectorBoundaries have been generated, via the boundariesGenerated boolean.
		 * @return True if the EffectorBoundaries have been generated.
		 **/
		inline bool hasBoundaries(){ return boundariesGenerated; }

		void generateBoundaries(double voxelSize);
		bool checkPath(const Vector3& begin, const Vector3& end);

		void moveTo(const Vector3& point, double maxAcceleration);
		void calibrateMotor(int motorIndex);
		bool checkSensor(int sensorIndex);
		bool calibrateMotors();
		void powerOff();
		void powerOn();
		Vector3 getEffectorLocation();

	private:
		std::string equipletName;
		rexos_datatypes::ModuleIdentifier identifier;
		bool isSimulated;
		/**
		 * @var InverseKinematicsModel* kinematics
		 * A pointer to the kinematics model used by the DeltaRobot.
		 **/
		InverseKinematicsModel* kinematics;

		rexos_delta_robot::DeltaRobotMeasures* deltaRobotMeasures;
		rexos_motor::StepperMotorProperties stepperMotorProperties;
	
		int calibrationBigStepFactor;
		/**
		 * @var StepperMotor* motors
		 * An array holding pointers to the three StepperMotors that are connected to the DeltaRobot. This array HAS to be of size 3.
		 **/
		std::vector<rexos_motor::MotorInterface*> motors;

		/**
		 * @var MotorManager* motorManager
		 * A pointer to the MotorManager that handles the movement for the DeltaRobot.
		 **/
		rexos_motor::MotorManager* motorManager;

		/**
		 * @var EffectorBoundaries* boundaries
		 * A pointer to the EffectorBoundaries of the DeltaRobot.
		 **/
		rexos_delta_robot::EffectorBoundaries* boundaries;

		/**
		 * @var Point3D<double> effectorLocation
		 * A 3D point in doubles that points to the location of the effector.
		 **/
		Vector3 effectorLocation;

		/**
		 * @var bool boundariesGenerated
		 * A boolean indicating whether the EffectorBoundaries have been generated or not.
		 **/
		bool boundariesGenerated;

		std::string modbusIp;
		int modbusPort;
		/**
		 * @var modbus_t* modbusIO
		 * A pointer to the TCP modbus connection for the IO controller.
		 **/
		modbus_t* modbusIO;
		/**
		 *
		 * @var ModbusController::ModbusController* modbus
		 * the modbuscontroller
		 **/
		rexos_modbus::ModbusController* modbus;

		/**
		 * @var int currentMotionSlot
		 * The motion slot currently in use. The deltarobot switches between these slots when moving.
		 **/
		int currentMotionSlot;

		int moveMotorUntilSensorIsOfValue(int motorIndex, rexos_motor::MotorRotation motorRotation, bool sensorValue);
		double getSpeedForRotation(double relativeAngle, double moveTime, double acceleration);
		double getAccelerationForRotation(double relativeAngle, double moveTime);
	};
}
