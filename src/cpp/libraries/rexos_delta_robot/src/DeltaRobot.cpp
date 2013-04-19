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

#include <sstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <cmath>

#include <rexos_datatypes/Point3D.h>
#include <rexos_delta_robot/EffectorBoundaries.h>
#include <rexos_delta_robot/InverseKinematics.h>
#include <rexos_delta_robot/InverseKinematicsException.h>
#include <rexos_delta_robot/DeltaRobot.h>
#include <rexos_motor/MotorException.h>
#include <rexos_motor/MotorInterface.h>
#include <rexos_utilities/Utilities.h>

namespace rexos_delta_robot{
    /**
     * Constructor of a deltarobot.
     * 
     * @param deltaRobotMeasures The measures of the deltarobot configuration in use.
     * @param motorManager The manager that allows all motors to be simultaneously activated.
     * @param motors The motor array with the three motor objects.
     * @param modbusIO The TCP modbus connection for the IO controller.
     **/
    DeltaRobot::DeltaRobot(rexos_datatypes::DeltaRobotMeasures& deltaRobotMeasures, rexos_motor::MotorManager* motorManager, rexos_motor::StepperMotor* (&motors)[3], modbus_t* modbusIO) :
        kinematics(NULL),
        motors(motors),
        motorManager(NULL),
        boundaries(NULL),
        effectorLocation(rexos_datatypes::Point3D<double>(0, 0, 0)), 
        boundariesGenerated(false),
        modbusIO(modbusIO),
        currentMotionSlot(1){

        if(modbusIO == NULL){
            throw std::runtime_error("Unable to open modbusIO");
        }
        kinematics = new InverseKinematics(deltaRobotMeasures);

        if(motorManager == NULL){
            throw std::runtime_error("No motorManager given");
        }
        this->motorManager = motorManager;
    }

    /**
     * Deconstructor of a deltarobot. Turns off the motors and deletes the kinematics model.
     **/
    DeltaRobot::~DeltaRobot(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
        delete kinematics;
    }
    
    /**
     * Generates the effectorBoundaries for the given voxelSize.
     *
     * @param voxelSize The size in millimeters of a side of a voxel in the boundaries.
     **/
    void DeltaRobot::generateBoundaries(double voxelSize){
        double motorMinAngles[3] = {Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MIN};
        double motorMaxAngles[3] = {Measures::MOTOR_ROT_MAX, Measures::MOTOR_ROT_MAX, Measures::MOTOR_ROT_MAX};
        boundaries = EffectorBoundaries::generateEffectorBoundaries((*kinematics), motorMinAngles, motorMaxAngles, voxelSize);
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
    bool DeltaRobot::isValidAngle(int motorIndex, double angle){
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
    bool DeltaRobot::checkPath(const rexos_datatypes::Point3D<double>& begin, const rexos_datatypes::Point3D<double>& end){
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
    double DeltaRobot::getAccelerationForRotation(double relativeAngle, double moveTime){
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
    double DeltaRobot::getSpeedForRotation(double relativeAngle, double moveTime, double acceleration){
        return (acceleration/2) * (moveTime - sqrt((moveTime * moveTime) - (4 * fabs(relativeAngle) / acceleration)));
    }

    /**
     * Makes the deltarobot move to a point.
     * 
     * @param point 3-dimensional point to move to.
     * @param maxAcceleration the acceleration in radians/s² that the motor with the biggest motion will accelerate at.
     **/
    void DeltaRobot::moveTo(const rexos_datatypes::Point3D<double>& point, double maxAcceleration){
        // check whether the motors are powered on.
        if(!motorManager->isPoweredOn()){
            throw rexos_motor::MotorException("motor drivers are not powered on");
        }

        if(effectorLocation == point){
            // The effector is already at the requested location, the method can be cut short.
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
        rexos_datatypes::MotorRotation* rotations[3];
        rotations[0] = new rexos_datatypes::MotorRotation();
        rotations[1] = new rexos_datatypes::MotorRotation();
        rotations[2] = new rexos_datatypes::MotorRotation();

        // Get the motor angles from the kinematics model
        try{
            kinematics->destinationPointToMotorRotations(point, rotations);
        } catch(InverseKinematicsException& ex){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
            throw ex;
        }

        // Check if the angles fit within the boundaries
        if(!isValidAngle(0, rotations[0]->angle) || !isValidAngle(1, rotations[1]->angle) || !isValidAngle(2, rotations[2]->angle)){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
            throw InverseKinematicsException("motion angles outside of valid range", point);
        }

        // Check if the path fits within the boundaries
        if(!boundaries->checkPath(effectorLocation, point)){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
            throw InverseKinematicsException("invalid path", point);
        }

        try{
            // An array to hold the relative angles for the motors
            double relativeAngles[3] = {0.0,0.0,0.0};

            // An array that indicates for each motor whether it moves in this motion or not.
            bool motorIsMoved[3] = {true, true, true};

            // Index for the motor with the biggest motion
            int motorWithBiggestMotion = 0;

            for(int i = 0; i < 3; i++){
                relativeAngles[i] = fabs(rotations[i]->angle - motors[i]->getCurrentAngle());
                if (relativeAngles[i] > relativeAngles[motorWithBiggestMotion]){
                    motorWithBiggestMotion = i;
                }

                if(relativeAngles[i] < rexos_motor::CRD514KD::MOTOR_STEP_ANGLE){
                    // motor does not have to move at all
                    motorIsMoved[i] = false;
                }
            }

            if(!(motorIsMoved[0] || motorIsMoved[1] || motorIsMoved[2])){
                // none of the motors have to move, method can be cut short
                delete rotations[0];
                delete rotations[1];
                delete rotations[2];
                return;
            }

             // switch currentMotionSlot
            currentMotionSlot++;
            if(currentMotionSlot > rexos_motor::CRD514KD::MOTION_SLOTS_USED){
                currentMotionSlot = 1;
            }

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
            for(int i = 0; i < 3; i++){
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
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
            throw ex;
        }

        delete rotations[0];
        delete rotations[1];
        delete rotations[2];
        effectorLocation = point;
    }

    /**
    * Reads calibration sensor and returns whether it is hit.
    * 
    * @param sensorIndex Index of the sensor. This corresponds to the motor index.
    * 
    * @return true if sensor is hit, false otherwise.
    **/
    bool DeltaRobot::checkSensor(int sensorIndex){
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
    int DeltaRobot::moveMotorUntilSensorIsOfValue(int motorIndex, rexos_datatypes::MotorRotation motorRotation, bool sensorValue){
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
    void DeltaRobot::calibrateMotor(int motorIndex){
        std::cout << "[DEBUG] Calibrating motor number " << motorIndex << std::endl;

        // Setup for incremental motion in big steps, to get to the sensor quickly.
        motors[motorIndex]->setIncrementalMode(1);
        rexos_datatypes::MotorRotation motorRotation;
        motorRotation.angle = -Measures::CALIBRATION_STEP_BIG;
        
        // Move to the sensor in large steps until it is pushed
        // actualAngleInSteps keeps track of how many motor steps the motor has moved. This is necessary to avoid accummulating errors.
        int actualAngleInSteps = moveMotorUntilSensorIsOfValue(motorIndex, motorRotation, true);

        // Move away from the sensor in big steps until it is no longer pushed.
        motorRotation.angle = -motorRotation.angle;
        actualAngleInSteps += moveMotorUntilSensorIsOfValue(motorIndex, motorRotation, false);
        
        // Move back to the sensor in small steps until it is pushed.
        motorRotation.angle = -Measures::CALIBRATION_STEP_SMALL;
        actualAngleInSteps += moveMotorUntilSensorIsOfValue(motorIndex, motorRotation, true);

        // calculate and set the deviation.
        double deviation = (actualAngleInSteps * rexos_motor::CRD514KD::MOTOR_STEP_ANGLE) + Measures::MOTORS_FROM_ZERO_TO_TOP_POSITION;
        motors[motorIndex]->setDeviationAndWriteMotorLimits(deviation);
        
        // Move back to the new 0.
        motors[motorIndex]->setAbsoluteMode(1);
        motorRotation.angle = 0;
        motors[motorIndex]->moveTo(motorRotation, 1);

        motors[motorIndex]->waitTillReady();
    }

    /**
    * Calibrates all three motors of the deltarobot by moving the motors upwards one by one.
    * After a motor is moved upwards, it is moved back to the 0 degrees state.
    * This function temporarily removes the limitations for the motorcontrollers.
    * 
    * @return true if the calibration was succesful. False otherwise (e.g. failure on sensors.)
    **/
    bool DeltaRobot::calibrateMotors(){       
        // Check the availability of the sensors
        bool sensorFailure = false;
        if(checkSensor(0)){
            std::cerr << "Sensor 0 failure (is the hardware connected?)" << std::endl;
            sensorFailure = true;
        }

        if(checkSensor(1)){
            std::cerr << "Sensor 1 failure (is the hardware connected?)" << std::endl;
            sensorFailure = true;
        }

        if(checkSensor(2)){
            std::cerr << "Sensor 2 failure (is the hardware connected?)" << std::endl;
            sensorFailure = true;
        }

        if(sensorFailure){
            return false;
        }

        // Return to base! Remove the deviation, we have to find the controller 0 point.
        rexos_datatypes::MotorRotation motorRotation;
        motorRotation.speed = 0.1;
        motorRotation.angle = 0;

        motors[0]->setDeviationAndWriteMotorLimits(0);
        motors[1]->setDeviationAndWriteMotorLimits(0);
        motors[2]->setDeviationAndWriteMotorLimits(0);

        motors[0]->writeRotationData(motorRotation, 1);
        motors[1]->writeRotationData(motorRotation, 1);
        motors[2]->writeRotationData(motorRotation, 1);
        motorManager->startMovement(1);

        motors[0]->waitTillReady();
        motors[1]->waitTillReady();
        motors[2]->waitTillReady();

        // Disable limitations
        motors[0]->disableAngleLimitations();
        motors[1]->disableAngleLimitations();
        motors[2]->disableAngleLimitations();
        
        // Calibrate motors
        calibrateMotor(0);
        calibrateMotor(1);
        calibrateMotor(2);

        // Enable angle limitations
        motors[0]->enableAngleLimitations();
        motors[1]->enableAngleLimitations();
        motors[2]->enableAngleLimitations();

        effectorLocation.x = 0;
        effectorLocation.y = 0;
        effectorLocation.z = -sqrt((Measures::ANKLE * Measures::ANKLE) - ((
                Measures::BASE + Measures::HIP - Measures::EFFECTOR) * (
                Measures::BASE + Measures::HIP - Measures::EFFECTOR))
        );
        std::cout << "[DEBUG] effector location z: " << effectorLocation.z << std::endl; 

        return true;
    }

    /**
     * Shuts down the deltarobot's hardware.
     **/
    void DeltaRobot::powerOff(void){
        if(motorManager->isPoweredOn()){
            motorManager->powerOff();
        }
    }

    /**
     * Turns on the deltarobot's hardware.
     **/
    void DeltaRobot::powerOn(void){
        if(!motorManager->isPoweredOn()){
            motorManager->powerOn();
        }
    }

    /**
     * Get the location of the midpoint of the effector.
     *
     * @return The coordinate for the midpoint of the effector.
     **/
    rexos_datatypes::Point3D<double>& DeltaRobot::getEffectorLocation(){
        return effectorLocation;
    }
}
