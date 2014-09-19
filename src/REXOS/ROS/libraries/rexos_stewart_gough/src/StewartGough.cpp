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
#include <stdlib.h> 
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
    StewartGough::StewartGough(Json::Value node) :
			motorManager(NULL),
			effectorLocation(rexos_datatypes::Point3D<double>(0, 0, 0)), 
			boundariesGenerated(false),
			currentMotionSlot(1){
		REXOS_INFO("StewartGough constructor entering...");
		readJSONNode(node);
		
		REXOS_INFO("Configuring Modbus...");
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


		sixAxisCalculations = new SixAxisCalculations(
			stewartGoughMeasures->hip,
			//stewartGoughMesuares->ankle,
			300.00, //ankle currently returns wrong value: 250.0, 19-05-2014
			stewartGoughMeasures->base,
			//stewartGoughMeasures->effector,
			55.0, //effector retuns wrong value: 44.x, 19-05-2014
			stewartGoughMeasures->maxAngleHipAnkle
			//0.26
			);

		REXOS_INFO_STREAM(" max angle: " << stewartGoughMeasures->maxAngleHipAnkle << std::endl); 
	

       // kinematics = new InverseKinematics;
		
		REXOS_INFO("end of constructor reached");
    }

    /**
     * Deconstructor of a stewart gough. Turns off the motors and deletes the kinematics model.
     **/
    StewartGough::~StewartGough(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
		delete sixAxisCalculations;
        //delete kinematics;
    }
    
	void StewartGough::readJSONNode(const Json::Value node) {
		modbusIp = node["modbusIp"].asString();
		REXOS_INFO_STREAM("found modbusIp " << modbusIp);
		
		modbusPort = node["modbusPort"].asInt();
		REXOS_INFO_STREAM("found modbusPort " << modbusPort);
		
		calibrationBigStepFactor = node["calibrationBigStepFactor"].asInt();
		REXOS_INFO_STREAM("found calibrationBigStepFactor " << calibrationBigStepFactor);
		
		Json::Value stepperMotorPropertiesNode = node["stepperMotorProperties"];
		stepperMotorProperties = new rexos_motor::StepperMotorProperties(stepperMotorPropertiesNode);
		REXOS_INFO_STREAM("found stepperMotorProperties");
		
		Json::Value stewartGoughNode = node["stewartGoughMeasures"];
		stewartGoughMeasures = new rexos_stewart_gough::StewartGoughMeasures(stewartGoughNode);
		REXOS_INFO_STREAM("found stewartGoughMeasures");
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
    bool StewartGough::checkPath(const rexos_datatypes::Point3D<double>& begin, const rexos_datatypes::Point3D<double>& end){
		return sixAxisCalculations->checkPath(
			SixAxisCalculations::Point3D(begin.x, begin.y, begin.z), 
			0, 0, 0, //Rotations start
			SixAxisCalculations::Point3D(end.x, end.y, end.z),
			0, 0, 0 //Rotations end
			);
			
		//return true;
        //return boundaries->checkPath(begin, end);
    }
	
	/**
     * Checks the path between two points.
     * 
     * @param begin The starting point.
     * @param end The end point.
     * 
     * @return if the path between two points is valid.
     **/
	bool StewartGough::checkPath(const rexos_datatypes::Point3D<double>& begin,
		const double beginRotationX,
		const double beginRotationY,
		const double beginRotationZ,
		const rexos_datatypes::Point3D<double>& end,
		const double endRotationX,
		const double endRotationY,
		const double endRotationZ){
    		return sixAxisCalculations->checkPath(
			SixAxisCalculations::Point3D(begin.x, begin.y, begin.z), 
			beginRotationX, beginRotationY, beginRotationZ, //Rotations start
			SixAxisCalculations::Point3D(end.x, end.y, end.z),
			endRotationX, endRotationY, endRotationZ //Rotations end
			);
	}
			
	
	

    /**
     * Gets the acceleration in radians/s² for a motor rotation with a certain relative angle and time, which is half acceleration and half deceleration (there is no period of constant speed).
     * 
     * @param moveTime the move time.
     * @param relativeAngle The relative angle
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
		moveTo(point, maxAcceleration, 0.0, 0.0, 0.0);
	}


    void StewartGough::moveTo(const rexos_datatypes::Point3D<double>& point, double maxAcceleration, double rotationX, double rotationY, double rotationZ){
		//maxAcceleration = 1;
		REXOS_INFO_STREAM("moveTo: point(x:" << point.x << ", y:" << point.y << ", z:" << point.z << ") rotation(x:" << rotationX << ", y:" << rotationY << ", z:" << rotationZ << ")" << std::endl);
		
		//rexos_datatypes::Point3D<double> roundedPoint(roundf(point.x), roundf(point.y), roundf(point.z));
		//std::cout << "moveTo (rounded): point(x:" << roundedPoint.x << ", y:" << roundedPoint.y << ", z:" << roundedPoint.z << ")" << std::endl;
		
		
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
        rexos_motor::MotorRotation* rotations[6];
		for(int i = 0; i < 6; i++){
			 rotations[i] = new rexos_motor::MotorRotation();
		}
		
        // Get the motor angles from the kinematics model
        try{
			
			
	
			SixAxisCalculations::EffectorMove effectorMove;
			
			
			
			effectorMove = sixAxisCalculations->getMotorAngles(SixAxisCalculations::Point3D(point.x, -point.y, point.z), rotationX, rotationY, rotationZ);
			
			
		//std::cout << "Rotation args from effectorMove: " << effectorMove.effectorRotationX << " " << effectorMove.effectorRotationY << " " << effectorMove.effectorRotationZ << std::endl;
		
			
			//calc.getAngles(angles, SixAxisCalculations::Point3D(point.x/10, point.y/10, point.z/10), rotationX, rotationY, rotationZ);
			if(!effectorMove.validMove){
				throw std::out_of_range("invalid angles"); 
			}
			for(int i = 0; i < 6;i++){
			
				//Swap 2 and 3
				if(i == 2){
					rotations[i]->angle = effectorMove.angles[3];
				} else if(i == 3){
					rotations[i]->angle = effectorMove.angles[2];
				} else
				//Swap 0 and 1
				if(i == 0){
					rotations[i]->angle = effectorMove.angles[1];
				} else if(i == 1){
					rotations[i]->angle = effectorMove.angles[0];
				} else {
					rotations[i]->angle = effectorMove.angles[i];
				}
				
				REXOS_INFO_STREAM("Angle for motor: " << i << " = " << effectorMove.angles[i] << std::endl);
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


		
        // Check if the path fits within the boundaries
        if(!checkPath(effectorLocation,
			currentEffectorRotationX,
			currentEffectorRotationY,
			currentEffectorRotationZ,
			point,
			rotationX,
			rotationY,
			rotationZ)){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
			delete rotations[3];
            delete rotations[4];
            delete rotations[5];
            throw std::out_of_range("invalid path");
        }
			


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
				
				//std::cout << "Hardware motor " << i << " rotation: " << rotations[i]->angle << std::endl;
				
                motors[i]->writeRotationData(*rotations[i], currentMotionSlot);
            }
			
			
			//long timer2 = rexos_utilities::timeNow();
            motorManager->startMovement(currentMotionSlot);
			//std::cout << "startMovement time: " << rexos_utilities::timeNow() - timer2 << "ms" << std::endl;
		
			
			
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
    MotorGroup StewartGough::moveMotorUntilSensorIsOfValue(int motorIndex1, int motorIndex2, rexos_motor::MotorRotation motorRotation1 ,rexos_motor::MotorRotation motorRotation2, bool sensorValue){
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
        // Setup for incremental motion in big steps, to get to the sensor quickly.
        motors[motorIndex1]->setIncrementalMode(1);
		motors[motorIndex2]->setIncrementalMode(1);
        rexos_motor::MotorRotation motorRotation1;
		rexos_motor::MotorRotation motorRotation2;
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
		
		
		
		//std::cout << "actual steps: " << std::endl;
		//std::cout << actualAngleInSteps1 << std::endl;
		//calculate and set the deviation.
		//std::cout << stewartGoughMeasures->motorFromZeroToTopAngle << std::endl;
        double deviation1 = (actualAngleInSteps1 * motors.at(motorIndex1)->getMicroStepAngle()) + stewartGoughMeasures->motorFromZeroToTopAngle;
        double deviation2 = (actualAngleInSteps2 * motors.at(motorIndex2)->getMicroStepAngle()) + stewartGoughMeasures->motorFromZeroToTopAngle;
		
		//std::cout << "passed 1" << std::endl;
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
		REXOS_INFO("Start motor calibration");
		
		// Check the availability of the sensors
		for(int i = 0; i < 6; i++){
			motorManager->powerOffSingleMotor(i);
		}
		
        bool sensorFailure = false;
		for(int i =0; i < 6; i++){
			if(checkSensor(i)){
				REXOS_ERROR_STREAM("Sensor " << i << "failure (is the hardware connected?)");
				sensorFailure = true;
			}
		}

        if(sensorFailure){
            return false;
        }

        // Return to base! Remove the deviation, we have to find the controller 0 point.
        rexos_motor::MotorRotation motorRotation;
        motorRotation.speed = 0.1;
        motorRotation.angle = 0;


		for(int i = 0; i < 6; i++){
			motorManager->powerOnSingleMotor(getMotorIndexByNumber(0 + i));
			getMotor(0 + i)->setDeviationAndWriteMotorLimits(0);
			getMotor(0 + i)->writeRotationData(motorRotation, 1);
			getMotor(0 + i)->startMovement(1);
			getMotor(0 + i)->waitTillReady();
			getMotor(0 + i)->disableAngleLimitations();
		}

		for(int i = 0; i < 6; i = i + 2){
			// Calibrate motors
			calibrateMotor(getMotorIndexByNumber(0 + i), getMotorIndexByNumber(1 + i));
			// Enable angle limitations
			getMotor(0 + i)->enableAngleLimitations();
			getMotor(1 + i)->enableAngleLimitations();
			
			REXOS_INFO_STREAM("Motors " << (0 + i) << " and " << (1 + i) << " calibrated");
		}
	
		
		
        effectorLocation.x = 0;
        effectorLocation.y = 0;
        effectorLocation.z = -270; // yet to be set 
		
		currentEffectorRotationX = 0;
		currentEffectorRotationY = 0;
		currentEffectorRotationZ = 0;
		
		
		
		
			
		REXOS_DEBUG_STREAM("Debuging sixaxis calculations" << std::endl);
		
		SixAxisCalculations::EffectorMove effectorMove = sixAxisCalculations->getMotorAngles(SixAxisCalculations::Point3D(0, 0, 300), 0, 0, 0);
		REXOS_DEBUG_STREAM("Move: " << effectorMove.moveTo << std::endl);
		for(int i = 0; i < 6; i++){
			REXOS_DEBUG_STREAM("Angle for motor " << i << " =" << effectorMove.angles[i] << std::endl);
		}
		
		effectorMove = sixAxisCalculations->getMotorAngles(SixAxisCalculations::Point3D(60, 0, 300), 0, 0, 0);
		REXOS_DEBUG_STREAM("Move: " << effectorMove.moveTo << std::endl);
		for(int i = 0; i < 6; i++){
			REXOS_DEBUG_STREAM("Angle for motor " << i << " =" << effectorMove.angles[i] << std::endl);
		}
		
		effectorMove = sixAxisCalculations->getMotorAngles(SixAxisCalculations::Point3D(0, 60, 300), 0, 0, 0);
		
		REXOS_DEBUG_STREAM("Move: " << effectorMove.moveTo << std::endl);
		for(int i = 0; i < 6; i++){
			REXOS_DEBUG_STREAM("Angle for motor " << i << " =" << effectorMove.angles[i] << std::endl);
		}
		
		
		/*
		double ac = 50;
		
		
		int x, y, z;
		double xr, yr, zr;
		
		int minX = -100;
		int maxX = 100; // -150 -- 150
		
		int minY = -100;
		int maxY = 100; // -150 -- 150
		
		int minZ = -300;
		int maxZ = -320; // -280 -- -340
		
		
		
		int minRotation = -40;
		int maxRotation = 40;
		
		
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -300), 0.9, 0, 0, 0);
		moveTo(rexos_datatypes::Point3D<double>(80, 0, -300), 0.9, 0, 0, 0);
		moveTo(rexos_datatypes::Point3D<double>(-80, 0, -300), 0.9, 0, 0, 0);
		moveTo(rexos_datatypes::Point3D<double>(0, 80, -300), 0.9, 0, 0, 0);
		moveTo(rexos_datatypes::Point3D<double>(0, -80, -300), 0.9, 0, 0, 0);
		
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -300), 0.9, 0, 0, 0);	
		for(int i = 0; i < 2000000; i++){
			
			x = (rand() % (maxX-minX))+minX;
			y = (rand() % (maxY-minY))+minY;
			z = (rand() % (maxZ-minZ))+minZ;
			
			
			xr = (rand() % (maxRotation-minRotation))+minRotation;
			yr = (rand() % (maxRotation-minRotation))+minRotation;
			zr = (rand() % (maxRotation-minRotation))+minRotation;

			xr = (xr/360.0) * M_PI;
			yr = (yr/360.0) * M_PI;
			zr = (zr/360.0) * M_PI;
			
			try {
				moveTo(rexos_datatypes::Point3D<double>(x, y, z), ac, xr, yr, zr);
			} catch(std::out_of_range& ex){
				std::cout << "Invalid position" << std::endl;
				//std::cout << "Position: " << x << ", " << y << ", " << z << std::endl; 
				//std::cout << "Rotation: " << xr << ", " << yr << ", " << zr << std::endl; 
			}
		
		}
		 */
		 
		 /*
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -280), 20, 0, 0, 0);
		
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(-40, 40, -280),  0.9, 0, 0, 0);
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(40, 40, -280),  0.9, 0, 0, 0);
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(40, -40, -280),  0.9, 0, 0, 0);
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(-40, -40, -280),  0.9, 0, 0, 0);
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -280),  0.9, 0, 0, 0);
		
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(0, -40, -280),  0.9, 0.174532925, 0, 0);
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(0, -40, -280),  0.9, -0.174532925, 0, 0);
		
		

		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -280),  0.9, 0, 0, 0);
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -280),  0.9, 0, 0.174532925, 0);
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -280),  0.9, 0, -0.174532925, 0);
		
		
		sleep(1);
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -280),  0.9, 0, 0, 0);
		*/
		
		/*
		sleep(2);
		moveTo(rexos_datatypes::Point3D<double>(0, -100, -350),  20, 0, 0, 0);
		sleep(2);	
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -350),  20, 0, 0, 0);
		sleep(2);
		moveTo(rexos_datatypes::Point3D<double>(100, 0, -350),  20, 0, 0, 0);
		sleep(2);
		moveTo(rexos_datatypes::Point3D<double>(-100, 0, -350),  20, 0, 0, 0);
		sleep(2);
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -350),  20, 0, 0, 0);
		sleep(2);
		moveTo(rexos_datatypes::Point3D<double>(80, 80, -350),  20, 0, 0, 0);
		sleep(2);
		moveTo(rexos_datatypes::Point3D<double>(-80, -80, -350),  20, 0, 0, 0);
		sleep(2);
		moveTo(rexos_datatypes::Point3D<double>(0, 0, -350),  20, 0, 0, 0);
		*/
		
		
        REXOS_DEBUG_STREAM("effector location z: " << effectorLocation.z << std::endl); 
		
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
		//REXOS_INFO("powering motors on");
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
	
	void StewartGough::deleteMotorRotationObjects(rexos_motor::MotorRotation* rotations[6]){
		for(int i = 0; i < 6; i++){
			delete rotations[i];
		}
	}
}
