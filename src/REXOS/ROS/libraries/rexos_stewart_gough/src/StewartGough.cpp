/**
 * @file DeltaRobot.cpp
 * @brief Symbolizes an entire deltarobot.
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

#include <sstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <cmath>

#include <rexos_datatypes/Point3D.h>
#include <rexos_stewart_gough/EffectorBoundaries.h>
#include <rexos_stewart_gough/InverseKinematics.h>
#include <rexos_stewart_gough/InverseKinematicsException.h>
#include <rexos_stewart_gough/StewartGough.h>
#include <rexos_motor/MotorException.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_utilities/Utilities.h>

#include "ros/ros.h"

namespace rexos_stewart_gough{
    /**
     * Constructor of a sixaxis robot.
     * 
     * @param steqartGoughMeasures The measures of the steqart gough configuration in use.
     * @param motorManager The manager that allows all motors to be simultaneously activated.
     * @param motors The motor array with the three motor objects.
     * @param modbusIO The TCP modbus connection for the IO controller.
     **/
    StewartGough::StewartGough(JSONNode node) :
			kinematics(NULL),
			motorManager(NULL),
			boundaries(NULL),
			effectorLocation(rexos_datatypes::Point3D<double>(0, 0, 0)), 
			boundariesGenerated(false),
			currentMotionSlot(1){
		ROS_INFO("StewartGough constructor entering...");
		readJSONNode(node);
		
		ROS_INFO("Configuring Modbus...");
		// Initialize modbus for IO controller
		modbusIO = modbus_new_tcp(modbusIp.c_str(), modbusPort);
		if(modbusIO == NULL){
			throw std::runtime_error("Unable to allocate libmodbus context");
		}
		if(modbus_connect(modbusIO) == -1) {
			throw std::runtime_error("Modbus connection to IO controller failed");
		}
		assert(modbusIO != NULL);
		
		// initialize motors
		modbus = new rexos_modbus::ModbusController(modbus_new_rtu(
			"/dev/ttyS0",
			rexos_motor::CRD514KD::RtuConfig::BAUDRATE,
			rexos_motor::CRD514KD::RtuConfig::PARITY,
			rexos_motor::CRD514KD::RtuConfig::DATA_BITS,
			rexos_motor::CRD514KD::RtuConfig::STOP_BITS));
		
		motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_0, *stepperMotorProperties));
		motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_1, *stepperMotorProperties));
		motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_2, *stepperMotorProperties));
		motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_3, *stepperMotorProperties));
		motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_4, *stepperMotorProperties));
		motors.push_back(new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_5, *stepperMotorProperties));

		motorManager = new rexos_motor::MotorManager(modbus, motors);

        kinematics = new InverseKinematics(*stewartGoughMeasures);
    }

    /**
     * Deconstructor of a stewart gough. Turns off the motors and deletes the kinematics model.
     **/
    StewartGough::~StewartGough(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
        delete kinematics;
    }
    
	void StewartGough::readJSONNode(const JSONNode node) {
		for(JSONNode::const_iterator it = node.begin(); it != node.end(); it++) {
			if(it->name() == "modbusIp"){
				modbusIp = it->as_string();
				ROS_INFO_STREAM("found modbusIp " << modbusIp);
			} else if(it->name() == "modbusPort"){
				modbusPort = it->as_int();
				ROS_INFO_STREAM("found modbusPort " << modbusPort);
			
			} else if(it->name() == "calibrationBigStepFactor"){
				calibrationBigStepFactor = it->as_int();
				ROS_INFO_STREAM("found calibrationBigStepFactor " << calibrationBigStepFactor);
			
			
			} else if(it->name() == "stepperMotorProperties"){
				JSONNode node = it->as_node();
				stepperMotorProperties = new rexos_motor::StepperMotorProperties(node);
				ROS_INFO_STREAM("found stepperMotorProperties");
			} else if(it->name() == "stewartGoughMeasures"){
				JSONNode node = it->as_node();
				stewartGoughMeasures = new rexos_datatypes::DeltaRobotMeasures(node);
				ROS_INFO_STREAM("found stewartGoughMeasures");
			} else {
				// some other property, ignore it
			}
		}
	}
    /**
     * Generates the effectorBoundaries for the given voxelSize.
     *
     * @param voxelSize The size in millimeters of a side of a voxel in the boundaries.
     **/
    void StewartGough::generateBoundaries(double voxelSize) {
		boundaries = EffectorBoundaries::generateEffectorBoundaries((*kinematics), stewartGoughMeasures, motors, voxelSize);
		boundariesGenerated = true;
    }

    /**
     * Checks the validity of an angle for a motor.
     *
     * @param motorIndex The index for the motor from 0 to (amount of motors - 1) to be checked for.
     * @param angle The angle in degrees where 0 degrees is directly opposite of the center of the imaginary circle for the engines.
     *
     * @return If the angle is valid for the motor.
     **/
    bool StewartGough::isValidAngle(int motorIndex, double angle) {
        assert(motorIndex >= 0 && motorIndex < 3);
        return angle > motors[motorIndex]->getMinAngle() && angle < motors[motorIndex]->getMaxAngle();
    }

    /**
     * Checks the path between two points.
     * 
     * @param begin The starting point.
     * @param end The end point.
     * 
     * @return if the path between two points is valid.
     **/
    bool StewartGough::checkPath(const rexos_datatypes::Point3D<double>& begin, const rexos_datatypes::Point3D<double>& end){
        return boundaries->checkPath(begin, end);
    }

    /**
     * Gets the acceleration in radians/s² for a motor rotation with a certain relative angle and time, which is half acceleration and half deceleration (there is no period of constant speed).
     * 
     * @param relativeAngle The relative angle
     * @param moveTime the move time.
     *
     * @return the acceleration in radians/s²
     **/
    double StewartGough::getAccelerationForRotation(double relativeAngle, double moveTime){
        return (4 * fabs(relativeAngle)) / (moveTime * moveTime);
    }

    /**
     * Gets the top speed in radians/s for a motor rotation with a certain relative angle, time and acceleration.
     * 
     * @param relativeAngle The relative angle
     * @param moveTime the move time.
     * @param acceleration the acceleration
     *
     * @return the speed in radians/s
     **/
    double StewartGough::getSpeedForRotation(double relativeAngle, double moveTime, double acceleration){
        return (acceleration/2) * (moveTime - sqrt((moveTime * moveTime) - (4 * fabs(relativeAngle) / acceleration)));
    }


    void StewartGough::moveTo(const rexos_datatypes::Point3D<double>& point, double maxAcceleration){
        
    }

    /**
    * Reads calibration sensor and returns whether it is hit.
    * 
    * @param sensorIndex Index of the sensor. This corresponds to the motor index.
    * 
    * @return true if sensor is hit, false otherwise.
    **/
    bool StewartGough::checkSensor(int sensorIndex){
        // The modbus library only reads
        uint16_t sensorRegister;
        int result;

        // Read register 8000 -- this register contains the values of the input sensors.
        result = modbus_read_registers(modbusIO, 8000, 1, &sensorRegister);
        if (result == -1){
            throw std::runtime_error(modbus_strerror(errno));
        }
        return (sensorRegister ^ 7) & 1 << sensorIndex;
    }

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
    int StewartGough::moveMotorUntilSensorIsOfValue(int motorIndex, rexos_datatypes::MotorRotation motorRotation, bool sensorValue){
        motors[motorIndex]->writeRotationData(motorRotation, 1, false);

        int steps = 0;
        do {
            motors[motorIndex]->startMovement(1);
            steps += (motorRotation.angle / rexos_motor::CRD514KD::MOTOR_STEP_ANGLE);  
        } while(checkSensor(motorIndex) != sensorValue);

        return steps;
    }

    /**
    * Calibrates a single motor by:
    * -# Moving it to the sensor in big steps until the sensor is pushed
    * -# Moving it away from the sensor in big steps until the sensor is no longer pushed
    * -# Moving back to the sensor in small steps until the sensor is pushed.
    * -# Using the moved steps to calculate the deviation
    * 
    * @param motorIndex Index of the motor to be calibrated. When standing in front of the robot looking towards it, 0 is the right motor, 1 is the front motor and 2 is the left motor.
    **/
    void StewartGough::calibrateMotor(int motorIndex){
        
    }

    /**
    * Calibrates all six motors of the deltarobot by moving the motors upwards one by one.
    * After a motor is moved upwards, it is moved back to the 0 degrees state.
    * This function temporarily removes the limitations for the motorcontrollers.
    * 
    * @return true if the calibration was succesful. False otherwise (e.g. failure on sensors.)
    **/
    bool StewartGough::calibrateMotors(){       
        std::cout << "calibration is yet to be implemented" << std::endl; 
        return true;
    }

    /**
     * Shuts down the stewart gough's hardware.
     **/
    void StewartGough::powerOff(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
    }

    /**
     * Turns on the stewart gough's hardware.
     **/
    void StewartGough::powerOn(void){
        if(!motorManager->isPoweredOn()){
            motorManager->powerOn();
        }
    }

    /**
     * Get the location of the midpoint of the effector.
     *
     * @return The coordinate for the midpoint of the effector.
     **/
    rexos_datatypes::Point3D<double>& StewartGough::getEffectorLocation(){
        return effectorLocation;
    }
}
