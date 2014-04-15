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
//#include <rexos_stewart_gough/EffectorBoundaries.h>

//#include <rexos_stewart_gough/InverseKinematics.h>
//#include <rexos_stewart_gough/InverseKinematicsException.h>
#include <rexos_stewart_gough/StewartGough.h>
#include <rexos_motor/MotorException.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_utilities/Utilities.h>

#include "ros/ros.h"

#include "rexos_stewart_gough/SixAxisCalculations.h"

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
			motorManager(NULL),
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

       // kinematics = new InverseKinematics;
		
		ROS_INFO("end of constructor reached");

		ROS_INFO("Reached the end of the constructor");
		ROS_INFO("end of constructor reached");
		ROS_INFO("Reached the end of the constructor");
    }

    /**
     * Deconstructor of a stewart gough. Turns off the motors and deletes the kinematics model.
     **/
    StewartGough::~StewartGough(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
        //delete kinematics;
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
        assert(motorIndex >= 0 && motorIndex < 6);
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
    //bool StewartGough::checkPath(const rexos_datatypes::Point3D<double>& begin, const rexos_datatypes::Point3D<double>& end){
        //return boundaries->checkPath(begin, end);
   // }

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
		moveTo(point, maxAcceleration, 0, 0, 0);
	}


    void StewartGough::moveTo(const rexos_datatypes::Point3D<double>& point, double maxAcceleration, double rotationX, double rotationY, double rotationZ){
		
		// check whether the motors are powered on.
		if(!motorManager->isPoweredOn()){
			throw rexos_motor::MotorException("motor drivers are not powered on");
		}

		//check if the requested location is the effectors current location,
		//if so the method can be cut short.
        if(effectorLocation == point
				&& currentEffectorRotationX == rotationX
				&& currentEffectorRotationY == rotationY
				&& currentEffectorRotationZ == rotationZ){
			return;
        }

        if(maxAcceleration > rexos_motor::CRD514KD::MOTOR_MAX_ACCELERATION){
            // The acceleration is too high, putting it down to the maximum CRD514KD acceleration.
            maxAcceleration = rexos_motor::CRD514KD::MOTOR_MAX_ACCELERATION;
        } else if(maxAcceleration < rexos_motor::CRD514KD::MOTOR_MIN_ACCELERATION){
            // The acceleration is too low, throwing an exception.
            throw std::out_of_range("maxAcceleration too low");            
        }

        // Create MotorRotation objects.
        rexos_datatypes::MotorRotation* rotations[6];
		for(int i = 0; i < 6; i++){
			 rotations[i] = new rexos_datatypes::MotorRotation();
		}
		
        // Get the motor angles from the kinematics model
        try{
			
			SixAxisCalculations calc;
			double angles[6];
			calc.getAngles(angles, SixAxisCalculations::Point3D(point.x/10, point.y/10, point.z/10), rotationX, rotationY, rotationZ);
			if(!calc.isValidMove(angles)){
				throw std::out_of_range("invalid angles"); 
			}
			for(int i = 0; i < 6;i++){
				rotations[i]->angle = angles[i];
			}
			
            //kinematics->destinationPointToMotorRotations(point, rotations);
        } catch(std::out_of_range& ex){
			deleteMotorRotationObjects(rotations);
            throw ex;
        }
		
		
		bool isInvalidAngles = false;
		
		for(int i = 0; i < 6; i++){
			if(!isValidAngle(i, rotations[i]->angle)){
				deleteMotorRotationObjects(rotations);
				throw std::out_of_range("motion angles outside of valid range");
			}
		}

		/*
		//TODO
        // Check if the path fits within the boundaries
        if(!boundaries->checkPath(effectorLocation, point)){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
			delete rotations[3];
            delete rotations[4];
            delete rotations[5];
            throw std::out_of_range("invalid path", point);
        }
	*/

        try{
            // An array to hold the relative angles for the motors
            double relativeAngles[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

            // An array that indicates for each motor whether it moves in this motion or not.
            bool motorIsMoved[6] = {true, true, true,true, true, true};

            // Index for the motor with the biggest motion
            int motorWithBiggestMotion = 0;

            for(int i = 0; i < 6; i++){
                relativeAngles[i] = fabs(rotations[i]->angle - motors[i]->getCurrentAngle());
                if (relativeAngles[i] > relativeAngles[motorWithBiggestMotion]){
                    motorWithBiggestMotion = i;
                }

                if(relativeAngles[i] < rexos_motor::CRD514KD::MOTOR_STEP_ANGLE){
                    // motor does not have to move at all
                    motorIsMoved[i] = false;
                }
            }

            if(!(	motorIsMoved[0] || motorIsMoved[1] || motorIsMoved[2] ||
					motorIsMoved[3] || motorIsMoved[4] || motorIsMoved[5]
			)){
                // none of the motors have to move, method can be cut short
				deleteMotorRotationObjects(rotations);
                return;
            }

             // switch currentMotionSlot
            //currentMotionSlot++;
            //if(currentMotionSlot > rexos_motor::CRD514KD::MOTION_SLOTS_USED){
			currentMotionSlot = 1;
            //}

            // Set the acceleration of the motor with the biggest motion to the given maximum.
            rotations[motorWithBiggestMotion]->acceleration = maxAcceleration;
            rotations[motorWithBiggestMotion]->deceleration = maxAcceleration;

            // Calculate the time the motion will take, based on the assumption that the motion is two-phase (half acceleration and half deceleration).
            // TODO: Take the motor's maximum speed into account.
            double moveTime;

            if(sqrt(relativeAngles[motorWithBiggestMotion] * rotations[motorWithBiggestMotion]->acceleration) > rexos_motor::CRD514KD::MOTOR_MAX_SPEED){
                // In case of a two-phase motion, the top speed would come out above the motor's maximum, so a three-phase motion must be made.
                rotations[motorWithBiggestMotion]->speed = rexos_motor::CRD514KD::MOTOR_MAX_SPEED;
                moveTime = (relativeAngles[motorWithBiggestMotion] / rotations[motorWithBiggestMotion]->speed) + (rotations[motorWithBiggestMotion]->speed / rotations[motorWithBiggestMotion]->acceleration);  
            } else {
                // The motion is fine as a two-phase motion.
                moveTime = 2 * sqrt(relativeAngles[motorWithBiggestMotion] / rotations[motorWithBiggestMotion]->acceleration);
            }
            
            // Set speed, and also the acceleration for the smaller motion motors
            for(int i = 0; i < 6; i++){
                rotations[i]->speed = rexos_motor::CRD514KD::MOTOR_MAX_SPEED;

                if(i != motorWithBiggestMotion){
                    if(motorIsMoved[i]){
                        rotations[i]->acceleration = getAccelerationForRotation(relativeAngles[i], moveTime);
                        rotations[i]->deceleration = rotations[i]->acceleration;  
                        if(rotations[i]->acceleration < rexos_motor::CRD514KD::MOTOR_MIN_ACCELERATION){
                            // The acceleration comes out too low, this means the motion cannot be half acceleration and half deceleration (without a consant speed phase).
                            // To make it comply with the move time, as well as the minimum acceleration requirements, we have to add a top speed.
                            rotations[i]->acceleration = rexos_motor::CRD514KD::MOTOR_MIN_ACCELERATION;
                            rotations[i]->deceleration = rexos_motor::CRD514KD::MOTOR_MIN_ACCELERATION;
                            rotations[i]->speed = getSpeedForRotation(relativeAngles[i], moveTime, rotations[i]->acceleration);
                        } else if(rotations[i]->acceleration > rexos_motor::CRD514KD::MOTOR_MAX_ACCELERATION){
                            throw std::out_of_range("acceleration too high");
                        }
                    } else {
                        rotations[i]->acceleration = rexos_motor::CRD514KD::MOTOR_MIN_ACCELERATION;
                        rotations[i]->deceleration = rexos_motor::CRD514KD::MOTOR_MIN_ACCELERATION;
                        rotations[i]->angle = motors[i]->getCurrentAngle();
                    }
                }
                motors[i]->writeRotationData(*rotations[i], currentMotionSlot);
            }

            motorManager->startMovement(currentMotionSlot);
        } catch(std::out_of_range& ex){
			deleteMotorRotationObjects(rotations);
            throw ex;
        }

        deleteMotorRotationObjects(rotations);
		
		currentEffectorRotationX = rotationX;
		currentEffectorRotationY = rotationY;
		currentEffectorRotationZ = rotationZ;
        effectorLocation = point;
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
    MotorGroup StewartGough::moveMotorUntilSensorIsOfValue(int motorIndex1, int motorIndex2, rexos_datatypes::MotorRotation motorRotation1 ,rexos_datatypes::MotorRotation motorRotation2, bool sensorValue){
		ROS_INFO("move Motor Until Sensor Is Of Value...");
        motors[motorIndex1]->writeRotationData(motorRotation1, 1, false);
		motors[motorIndex2]->writeRotationData(motorRotation2, 1, false);
		bool done1 = false;
		bool done2 = false;
        MotorGroup motorGroupSteps(0,0);
		
		while(1){
			done1 |= (checkSensor(motorMap[motorIndex1].sensor) == sensorValue);
			done2 |= (checkSensor(motorMap[motorIndex2].sensor) == sensorValue);
			
			if(!done1){
				motors[motorIndex1]->startMovement(1);
				motorGroupSteps.motorIndex1 += (motorRotation1.angle / rexos_motor::CRD514KD::MOTOR_STEP_ANGLE); 
			}
			
			if(!done2) {
				motors[motorIndex2]->startMovement(1);
				motorGroupSteps.motorIndex2 += (motorRotation2.angle / rexos_motor::CRD514KD::MOTOR_STEP_ANGLE); 
			}
			
			if(done1 == true && done2 == true){
				return motorGroupSteps;
			}
		}
		return motorGroupSteps;
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
    void StewartGough::calibrateMotor(int motorIndex1, int motorIndex2){
       std::cout << "[DEBUG] Calibrating motor number " << motorIndex1 << std::endl;

        // Setup for incremental motion in big steps, to get to the sensor quickly.
        motors[motorIndex1]->setIncrementalMode(1);
		motors[motorIndex2]->setIncrementalMode(1);
        rexos_datatypes::MotorRotation motorRotation1;
		rexos_datatypes::MotorRotation motorRotation2;
        motorRotation1.angle = -motors.at(motorIndex1)->getMicroStepAngle() * calibrationBigStepFactor;
		motorRotation2.angle = -motors.at(motorIndex2)->getMicroStepAngle() * calibrationBigStepFactor;
        
        // Move to the sensor in large steps until it is pushed
        // actualAngleInSteps keeps track of how many motor steps the motor has moved. This is necessary to avoid accummulating errors.
        MotorGroup actualAnglesInSteps = moveMotorUntilSensorIsOfValue(motorIndex1, motorIndex2, motorRotation1,motorRotation2, true);
		
		int actualAngleInSteps1 = actualAnglesInSteps.motorIndex1;
		int actualAngleInSteps2 = actualAnglesInSteps.motorIndex2;
		
        // Move away from the sensor in big steps until it is no longer pushed.
        motorRotation1.angle = -motorRotation1.angle;
		motorRotation2.angle = -motorRotation2.angle;
        actualAnglesInSteps = moveMotorUntilSensorIsOfValue(motorIndex1,motorIndex2, motorRotation1,motorRotation2, false);
		actualAngleInSteps1 += actualAnglesInSteps.motorIndex1;
		actualAngleInSteps2 += actualAnglesInSteps.motorIndex2;

        // Move back to the sensor in small steps until it is pushed.
        motorRotation1.angle = -motors.at(motorIndex1)->getMicroStepAngle();
		motorRotation2.angle = -motors.at(motorIndex2)->getMicroStepAngle();
        actualAnglesInSteps = moveMotorUntilSensorIsOfValue(motorIndex1,motorIndex2, motorRotation1,motorRotation2, true);
		actualAngleInSteps1 += actualAnglesInSteps.motorIndex1;
		actualAngleInSteps2 += actualAnglesInSteps.motorIndex2;
		
		
		
		std::cout << "actual steps: " << std::endl;
		std::cout << actualAngleInSteps1 << std::endl;
		//calculate and set the deviation.
		//std::cout << stewartGoughMeasures->motorFromZeroToTopAngle << std::endl;
        double deviation1 = (actualAngleInSteps1 * motors.at(motorIndex1)->getMicroStepAngle()) + stewartGoughMeasures->motorFromZeroToTopAngle;
        double deviation2 = (actualAngleInSteps2 * motors.at(motorIndex2)->getMicroStepAngle()) + stewartGoughMeasures->motorFromZeroToTopAngle;
		
		std::cout << "passed 1" << std::endl;
		motors[motorIndex1]->setDeviationAndWriteMotorLimits(deviation1);
		motors[motorIndex2]->setDeviationAndWriteMotorLimits(deviation2);
        // Move back to the new 0.
		
        motors[motorIndex1]->setAbsoluteMode(1);
		motors[motorIndex2]->setAbsoluteMode(1);
        motorRotation1.angle = 0;
		motorRotation2.angle = 0;
        motors[motorIndex1]->moveTo(motorRotation1, 1);
		motors[motorIndex2]->moveTo(motorRotation2, 1);
		
        motors[motorIndex1]->waitTillReady();
		motors[motorIndex2]->waitTillReady();
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
		
		for(int i = 0; i < 6; i++){
			motorManager->powerOffSingleMotor(i);
		}
		
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


		for(int i = 0; i < 6; i++){
			motorManager->powerOnSingleMotor(getMotorIndexByNumber(0 + i));
			
			getMotor(0 + i)->setDeviationAndWriteMotorLimits(0);
			getMotor(0 + i)->writeRotationData(motorRotation, 1);
			
			
			getMotor(0 + i)->startMovement(1);
			//motorManager->startMovement(1);
			//std::cout << "did start movement" << std::endl;
			
			getMotor(0 + i)->waitTillReady();
			getMotor(0 + i)->disableAngleLimitations();
		}

		for(int i = 0; i < 6; i = i + 2){
			/*
			motorManager->powerOnSingleMotor(getMotorIndexByNumber(0 + i));
			motorManager->powerOnSingleMotor(getMotorIndexByNumber(1 + i));
			
			//std::cout << "powered on 1 and 2" << std::endl;
			getMotor(0 + i)->setDeviationAndWriteMotorLimits(0);
			getMotor(0 + i)->writeRotationData(motorRotation, 1);
			getMotor(1 + i)->setDeviationAndWriteMotorLimits(0);
			getMotor(1 + i)->writeRotationData(motorRotation, 1);
			//std::cout << "wrote diviation and rotation data" << std::endl;
			
			//getMotor(0 + i)->startMovement(1);
			motorManager->startMovement(1);
			//std::cout << "did start movement" << std::endl;
			
			getMotor(0 + i)->waitTillReady();
			getMotor(1 + i)->waitTillReady();
			
			getMotor(0 + i)->disableAngleLimitations();
			getMotor(1 + i)->disableAngleLimitations();
			*/
			
			// Calibrate motors
			calibrateMotor(getMotorIndexByNumber(0 + i), getMotorIndexByNumber(1 + i));
			// Enable angle limitations
			getMotor(0 + i)->enableAngleLimitations();
			getMotor(1 + i)->enableAngleLimitations();
			
			std::cout << "Motors " << (0 + i) << " and " << (1 + i) << " done calibrating!" << std::endl;
		}
	
		
		
        effectorLocation.x = 0;
        effectorLocation.y = 0;
        effectorLocation.z = -270; // yet to be set 
		
		currentEffectorRotationX = 0;
		currentEffectorRotationY = 0;
		currentEffectorRotationZ = 0;
		
		
		//moveTo(rexos_datatypes::Point3D<double>(0, 0, -280), 10, 0, 0, 30);
		
		//sleep(3);
		
		//moveTo(rexos_datatypes::Point3D<double>(0, 0, -280), 10, 0, 0, 0);
		
		//sleep(3);
		
		//moveTo(rexos_datatypes::Point3D<double>(0, 0, -280), 10, 0, 0, -30);
		
		
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
	
	double StewartGough::getEffectorRotationX(){
		return currentEffectorRotationX;
	}
	
	double StewartGough::getEffectorRotationY(){
		return currentEffectorRotationY;
	}
	
	double StewartGough::getEffectorRotationZ(){
		return currentEffectorRotationZ;
	}
	
	rexos_motor::StepperMotor* StewartGough::getMotor(int number){
		return motors[getMotorIndexByNumber(number)];
	}
	
	int StewartGough::getMotorIndexByNumber(int number){
		return motorMap[number].motor;
	}
	
	void StewartGough::deleteMotorRotationObjects(rexos_datatypes::MotorRotation* rotations[6]){
		for(int i = 0; i < 6; i++){
			delete rotations[i];
		}
	}
}
