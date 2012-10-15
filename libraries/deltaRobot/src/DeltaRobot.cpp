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

#include <DataTypes/Point3D.h>
#include <Motor/MotorInterface.h>
#include <DeltaRobot/EffectorBoundaries.h>
#include <DeltaRobot/InverseKinematics.h>
#include <DeltaRobot/InverseKinematicsException.h>
#include <DeltaRobot/DeltaRobot.h>
#include <Motor/MotorException.h>
#include <Utilities/Utilities.h>

namespace DeltaRobot{
    /**
     * Constructor of a DeltaRobot.
     * 
     * @param deltaRobotMeasures The measures of the deltarobot configuration in use.
     * @param motorManager The manager that allows all motors to be simultaneously activated.
     * @param motors The motor array with the three motor objects.
     * @param modbusIO The TCP modbus connection for the IO controller.
     **/
    DeltaRobot::DeltaRobot(DataTypes::DeltaRobotMeasures& deltaRobotMeasures, Motor::MotorManager* motorManager, Motor::StepperMotor* (&motors)[3], modbus_t* modbusIO) :
        motors(motors),
        effectorLocation(DataTypes::Point3D<double>(0, 0, 0)), 
        boundariesGenerated(false),
        modbusIO(modbusIO){

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
       TODO: comment
     **/
    void DeltaRobot::generateBoundaries(double voxelSize){
        boundaries = EffectorBoundaries::generateEffectorBoundaries((*kinematics), motors, voxelSize);
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
    bool DeltaRobot::checkPath(const DataTypes::Point3D<double>& begin, const DataTypes::Point3D<double>& end){
        return boundaries->checkPath(begin, end);
    }

    /**
     * Makes the deltarobot move to a point.
     * 
     * @param point 3-dimensional point to move to.
     * @param speed Movement speed in millimeters per second.
     **/
    void DeltaRobot::moveTo(const DataTypes::Point3D<double>& point, double speed){
        // TODO: Some comments in this function would be nice.
        if(!motorManager->isPoweredOn()){
            throw Motor::MotorException("motor drivers are not powered on");
        }

        DataTypes::MotorRotation<double>* rotations[3];
        rotations[0] = new DataTypes::MotorRotation<double>();
        rotations[1] = new DataTypes::MotorRotation<double>();
        rotations[2] = new DataTypes::MotorRotation<double>();

        rotations[0]->speed = speed;
        rotations[1]->speed = speed;
        rotations[2]->speed = speed;

        try{
            kinematics->pointToMotion(point, rotations);
        } catch(InverseKinematicsException& ex){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
            throw ex;
        }

        if(!isValidAngle(0, rotations[0]->angle) || !isValidAngle(1, rotations[1]->angle) || !isValidAngle(2, rotations[2]->angle)){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
            throw InverseKinematicsException("motion angles outside of valid range", point);
        }

        if(!boundaries->checkPath(effectorLocation, point)){
            delete rotations[0];
            delete rotations[1];
            delete rotations[2];
            throw InverseKinematicsException("invalid path", point);
        }

        double moveTime = point.distance(effectorLocation) / speed;
        try{
            motors[0]->moveToWithin(*rotations[0], moveTime, false);
            motors[1]->moveToWithin(*rotations[1], moveTime, false);
            motors[2]->moveToWithin(*rotations[2], moveTime, false);
            motorManager->startMovement();
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
    * Calibrates a single motor by moving the motor upwards till the calibration sensor is pushed.
    * 
    * @param motorIndex Index of the motor to be calibrated. When standing in front of the robot looking towards it, 0 is the right motor, 1 is the front motor and 2 is the left motor.
    **/
    void DeltaRobot::calibrateMotor(int motorIndex){
        std::cout << "[DEBUG] Calibrating motor number " << motorIndex << std::endl;
        
        // Starting point of calibration
        DataTypes::MotorRotation<double> motorRotation;
        motorRotation.speed = 1;
        motorRotation.acceleration = 360;
        motorRotation.deceleration = 360;
        motorRotation.angle = 0;

        // Move motor upwards till the calibration sensor is pushed
        do{
            motorRotation.angle -= Utilities::degreesToRadians(Motor::CRD514KD::MOTOR_STEP_IN_DEGREES);
            motors[motorIndex]->moveTo(motorRotation);

            usleep(25000);
        } while(!checkSensor(motorIndex));

        double deviation = motorRotation.angle + Measures::MOTORS_DEVIATION;

        // Set deviation to the calculated value.
        motors[motorIndex]->setDeviation(deviation);

        motorRotation.angle = 0;
        motors[motorIndex]->moveTo(motorRotation);

        // Wait for steady
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
        DataTypes::MotorRotation<double> motorRotation;
        motorRotation.speed = 0.1;
        motorRotation.angle = 0;

        motors[0]->setDeviation(0);
        motors[1]->setDeviation(0);
        motors[2]->setDeviation(0);

        motors[0]->writeRotationData(motorRotation);
        motors[1]->writeRotationData(motorRotation);
        motors[2]->writeRotationData(motorRotation);
        motorManager->startMovement();

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

        // Set limitations
        motors[0]->setMotorLimits(Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MAX);
        motors[1]->setMotorLimits(Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MAX);
        motors[2]->setMotorLimits(Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MAX);

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
    DataTypes::Point3D<double>& DeltaRobot::getEffectorLocation(){
        return effectorLocation;
    }
}
