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
#include <rexos_datatypes/Point3D.h>
#include <rexos_datatypes/DeltaRobotMeasures.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_motor/StepperMotor.h>
#include <rexos_motor/MotorManager.h>
#include <rexos_delta_robot/EffectorBoundaries.h>

namespace rexos_delta_robot{
	class InverseKinematicsModel;
	/**
	 * A class that symbolizes an entire deltarobot.
	 **/
	class DeltaRobot{
	public:
		DeltaRobot(rexos_datatypes::DeltaRobotMeasures& deltaRobotMeasures, rexos_motor::MotorManager* motorManager, rexos_motor::StepperMotor* (&motors)[3], modbus_t* modbusIO);
		~DeltaRobot();

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
		bool checkPath(const rexos_datatypes::Point3D<double>& begin, const rexos_datatypes::Point3D<double>& end);

		void moveTo(const rexos_datatypes::Point3D<double>& point, double maxAcceleration);
		void calibrateMotor(int motorIndex);
		bool checkSensor(int sensorIndex);
		bool calibrateMotors();
		void powerOff();
		void powerOn();
		rexos_datatypes::Point3D<double>& getEffectorLocation();

	private:
		/**
		 * @var InverseKinematicsModel* kinematics
		 * A pointer to the kinematics model used by the DeltaRobot.
		 **/
		InverseKinematicsModel* kinematics;

		/**
		 * @var StepperMotor* motors
		 * An array holding pointers to the three StepperMotors that are connected to the DeltaRobot. This array HAS to be of size 3.
		 **/
		rexos_motor::StepperMotor* (&motors)[3];

		/**
		 * @var MotorManager* motorManager
		 * A pointer to the MotorManager that handles the movement for the DeltaRobot.
		 **/
		rexos_motor::MotorManager* motorManager;

		/**
		 * @var EffectorBoundaries* boundaries
		 * A pointer to the EffectorBoundaries of the DeltaRobot.
		 **/
		EffectorBoundaries* boundaries;

		/**
		 * @var Point3D<double> effectorLocation
		 * A 3D point in doubles that points to the location of the effector.
		 **/
		rexos_datatypes::Point3D<double> effectorLocation;

		/**
		 * @var bool boundariesGenerated
		 * A boolean indicating whether the EffectorBoundaries have been generated or not.
		 **/
		bool boundariesGenerated;

		/**
		 * @var modbus_t* modbusIO
		 * A pointer to the TCP modbus connection for the IO controller.
		 **/
		modbus_t* modbusIO;

		/**
		 * @var int currentMotionSlot
		 * The motion slot currently in use. The deltarobot switches between these slots when moving.
		 **/
		int currentMotionSlot;

		bool isValidAngle(int motorIndex, double angle);
		int moveMotorUntilSensorIsOfValue(int motorIndex, rexos_datatypes::MotorRotation motorRotation, bool sensorValue);
		double getSpeedForRotation(double relativeAngle, double moveTime, double acceleration);
		double getAccelerationForRotation(double relativeAngle, double moveTime);
	};
}
