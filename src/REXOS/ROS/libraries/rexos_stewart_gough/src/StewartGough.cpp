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
		
		motorMap[0] = MotorMap(0,3);
		motorMap[1] = MotorMap(1,2);
		motorMap[2] = MotorMap(2,1);
		motorMap[3] = MotorMap(3,0);
		motorMap[4] = MotorMap(4,4);
		motorMap[5] = MotorMap(5,5);

		//int motorMap1[][2] = {{1, 2}, {1, 2}};
		
		motorManager = new rexos_motor::MotorManager(modbus, motors);

        kinematics = new InverseKinematics;
<<<<<<< HEAD
		
		ROS_INFO("end of constructor reached");
=======
		ROS_INFO("Reached the end of the constructor");
>>>>>>> 98831652dc41662521587ab9abb9de3ffcf89903
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
				stewartGoughMeasures = new rexos_datatypes::StewartGoughMeasures(node);
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
     *
    void StewartGough::generateBoundaries(double voxelSize) {
		boundaries = EffectorBoundaries::generateEffectorBoundaries((*kinematics), stewartGoughMeasures, motors, voxelSize);
		boundariesGenerated = true;
    }
*/
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
        return (sensorRegister ^ 63) & 1 << sensorIndex;
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
		ROS_INFO("move Motor Until Sensor Is Of Value...");
        motors[motorIndex]->writeRotationData(motorRotation, 1, false);

        int steps = 0;
		ROS_INFO("check1");
        do {
			ROS_INFO("check2");
            motors[motorIndex]->startMovement(1);
			
            steps += (motorRotation.angle / rexos_motor::CRD514KD::MOTOR_STEP_ANGLE);  
        } while(checkSensor(motorMap[motorIndex].sensor) != sensorValue);

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
       std::cout << "[DEBUG] Calibrating motor number " << motorIndex << std::endl;

        // Setup for incremental motion in big steps, to get to the sensor quickly.
        motors[motorIndex]->setIncrementalMode(1);
        rexos_datatypes::MotorRotation motorRotation;
        motorRotation.angle = -motors.at(motorIndex)->getMicroStepAngle() * calibrationBigStepFactor;
        
        // Move to the sensor in large steps until it is pushed
        // actualAngleInSteps keeps track of how many motor steps the motor has moved. This is necessary to avoid accummulating errors.
        int actualAngleInSteps = moveMotorUntilSensorIsOfValue(motorIndex, motorRotation, true);

        // Move away from the sensor in big steps until it is no longer pushed.
        motorRotation.angle = -motorRotation.angle;
        actualAngleInSteps += moveMotorUntilSensorIsOfValue(motorIndex, motorRotation, false);
        
        // Move back to the sensor in small steps until it is pushed.
        motorRotation.angle = -motors.at(motorIndex)->getMicroStepAngle();
        actualAngleInSteps += moveMotorUntilSensorIsOfValue(motorIndex, motorRotation, true);

		//calculate and set the deviation.
        //double deviation = (actualAngleInSteps * motors.at(motorIndex)->getMicroStepAngle()) + deltaRobotMeasures->motorFromZeroToTopAngle;
        //motors[motorIndex]->setDeviationAndWriteMotorLimits(deviation);
        
        // Move back to the new 0.
		
        motors[motorIndex]->setAbsoluteMode(1);
        motorRotation.angle = 0;
        motors[motorIndex]->moveTo(motorRotation, 1);

        motors[motorIndex]->waitTillReady();
    }

    /**
    * Calibrates all six motors of the deltarobot by moving the motors upwards one by one.
    * After a motor is moved upwards, it is moved back to the 0 degrees state.
    * This function temporarily removes the limitations for the motorcontrollers.
    * 
    * @return true if the calibration was succesful. False otherwise (e.g. failure on sensors.)
    **/
    bool StewartGough::calibrateMotors(){       
        std::cout << "calibrating" << std::endl;
		// Check the availability of the sensors

        bool sensorFailure = false;
		
		for(int i =0; i < 6; i++){
			if(checkSensor(i)){
				std::cerr << "Sensor 0 failure (is the hardware connected?)" << std::endl;
				sensorFailure = true;
			}
		}

        if(sensorFailure){
            return false;
        }

        // Return to base! Remove the deviation, we have to find the controller 0 point.
        rexos_datatypes::MotorRotation motorRotation;
        motorRotation.speed = 0.1;
        motorRotation.angle = 0;


		for(int i =0; i < 6; i++){

			motors[i]->setDeviationAndWriteMotorLimits(0);
			//motors[1]->setDeviationAndWriteMotorLimits(0);
			//motors[2]->setDeviationAndWriteMotorLimits(0);

			motors[i]->writeRotationData(motorRotation, 1);
			//motors[1]->writeRotationData(motorRotation, 1);
			//motors[2]->writeRotationData(motorRotation, 1);
		}
		
		motorManager->startMovement(1);
		
		
		for(int i =0; i < 6; i++){
			

			motors[i]->waitTillReady();
			//motors[1]->waitTillReady();
			//motors[2]->waitTillReady();

			// Disable limitations
			motors[i]->disableAngleLimitations();
			//motors[1]->disableAngleLimitations();
			//motors[2]->disableAngleLimitations();
			
			// Calibrate motors
			calibrateMotor(i);
			//calibrateMotor(1);
			//calibrateMotor(2);

			// Enable angle limitations
			motors[i]->enableAngleLimitations();
			//motors[1]->enableAngleLimitations();
			//motors[2]->enableAngleLimitations();
		}
		
        effectorLocation.x = 0;
        effectorLocation.y = 0;
        effectorLocation.z = 0; // yet to be set
        std::cout << "[DEBUG] effector location z: " << effectorLocation.z << std::endl; 
		
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
		ROS_INFO("powering motors on");
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
