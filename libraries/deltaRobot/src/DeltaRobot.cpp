//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        deltarobot.cpp
// File:           symbolizes an entire deltarobot
// Description:
// Author:      1.0 Lukas Vermond & Kasper van Nieuwland
//              1.1 Koen Braham
// Notes:          
//
// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************


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

/**
 * DeltaRobot.cpp -> This class symbolises an entire deltarobot
 **/

namespace DeltaRobot
{
    /**
     * Constructor
     * @param kinematics kinematics model that will be used to convert points to motions
     * @param motors implementation of motor interface that will be used to communicate with the motors. MUST BE EXACTLY 3.
     **/
    DeltaRobot::DeltaRobot(DataTypes::DeltaRobotMeasures& drm, Motor::MotorManager* motorManager, Motor::StepperMotor* (&motors)[3]) :
        motors(motors),
         effectorLocation(DataTypes::Point3D<double>(0, 0, -161.9)), 
         boundariesGenerated(false)
    {
        kinematics = new InverseKinematics(drm);
        this->motorManager = motorManager;
    }

    DeltaRobot::~DeltaRobot(void)
    {
        if(motorManager->isPoweredOn())
        {
            motorManager->powerOff();
        }
        delete kinematics;
    }
    
    void DeltaRobot::generateBoundaries(double voxelSize){
        boundaries = EffectorBoundaries::generateEffectorBoundaries((*kinematics), motors, voxelSize);
        boundariesGenerated = true;
    }

    bool DeltaRobot::isValidAngle(int motorIndex, double angle)
    {
        assert(motorIndex >= 0 && motorIndex < 3);
        return angle > motors[motorIndex]->getMinAngle() && angle < motors[motorIndex]->getMaxAngle();
    }

    /**
     * Checks the path between two points
     * @param begin start point
     * @param end finish point
     **/
    bool DeltaRobot::checkPath(const DataTypes::Point3D<double>& begin,const DataTypes::Point3D<double>& end)
    {
        return boundaries->checkPath(begin, end);
    }

    /**
     * Makes the deltarobot move to a point
     * @param p 3-dimensional point to move to
     * @param speed movement speed in millimeters per second
     * @param async motions will be stored in a queue for later execution if true
     **/
    void DeltaRobot::moveTo(const DataTypes::Point3D<double>& p, double speed)
    {
        if(!motorManager->isPoweredOn())
        {
            throw Motor::MotorException("motor drivers are not powered on");
        }

        DataTypes::DeltaRobotRotation drr;
        drr.rotations[0].speed = speed;
        drr.rotations[1].speed = speed;
        drr.rotations[2].speed = speed;

        try{
            kinematics->pointToMotion(p, drr);
        }
        catch(InverseKinematicsException& ex){
            throw ex;
        }

        std::cout << "Angles\t" << drr.rotations[0].angle << "\t" << drr.rotations[1].angle << "\t" << drr.rotations[2].angle << std::endl;
        if(
            !isValidAngle(0, drr.rotations[0].angle) ||
            !isValidAngle(1, drr.rotations[1].angle) ||
            !isValidAngle(2, drr.rotations[2].angle))
        {
            throw InverseKinematicsException("motion angles outside of valid range", p);
        }

        if(!boundaries->checkPath(effectorLocation, p))
        {
            throw InverseKinematicsException("invalid path", p);
        }

        std::cout << "moveTime coord1 (" << p.x << " " << p.y << " " << p.z << ") coord2 (" << effectorLocation.x << " " << effectorLocation.y << " " << effectorLocation.z << ") speed " << speed << " distance " << p.distance(effectorLocation) << std::endl;
        double moveTime = p.distance(effectorLocation) / speed;

        try
        {

            motors[0]->moveToWithin(drr.rotations[0], moveTime, false);
            motors[1]->moveToWithin(drr.rotations[1], moveTime, false);
            motors[2]->moveToWithin(drr.rotations[2], moveTime, false);
            motorManager->startMovement();
        }
        catch(std::out_of_range& ex) { throw ex; }


        effectorLocation = p;
    }

    /**
    * Reads calibration sensor and returns whether it is hit.
    * @param modbus The TCP modbus connection for IO controller.
    * @param sensorIndex index of the sensor. This corresponds to the motor index.
    * @return True if sensor is hit, false otherwise.
    **/
    bool DeltaRobot::checkSensor(modbus_t* modbus, int sensorIndex){
        // The modbus library only reads
        uint16_t sensorRegister;
        int result;

        // Read register 8000 -- this register contains the values of the input sensors.
        result = modbus_read_registers(modbus, 8000, 1, &sensorRegister);
        if (result == -1) {
            throw std::runtime_error(modbus_strerror(errno));
        }
        return (sensorRegister ^ 7) & 1 << sensorIndex;
    }

    /**
    * @brief Calibrates a single motor by moving the motor upwards till the calibration sensor is pushed.
    * @param modbus The TCP modbus connection for IO controller.
    * @param motors The StepperMotor class controlling the 3 deltarobot motors.
    * @param motorIndex Index of the motor to be calibrated. When standing in front of the robot looking towards it, 0 is the right motor, 1 is the front motor and 2 is the left motor.
    **/
    void DeltaRobot::calibrateMotor(modbus_t* modbus, int motorIndex) {
        std::cout << "Calibrating motor number " << motorIndex << std::endl;
        
        // Starting point of calibration
        DataTypes::MotorRotation<double> mr;
        mr.speed = 1;
        mr.acceleration = 360;
        mr.deceleration = 360;
        mr.angle = 0;

        // Move motor upwards till the calibration sensor is pushed
        do {
            mr.angle -= Utilities::rad(Motor::CRD514KD::MOTOR_STEP_IN_DEGREES);
            motors[motorIndex]->moveTo(mr);

            usleep(25000);
        } while(!checkSensor(modbus, motorIndex));

        //
        mr.angle += Measures::MOTORS_DEVIATION;
        motors[motorIndex]->moveTo(mr);

        motors[motorIndex]->resetCounter();

        mr.angle = 0;
        motors[motorIndex]->moveTo(mr);
    }

    /**
    * @brief Calibrates all three motors of the deltarobot by moving the motors upwards one by one.
    * After a motor is moved upwards, it is moved back to the 0 degrees state.
    * This function temporarily removes the limitations for the motorcontrollers.
    * @param modbus The TCP modbus connection for IO controller.
    * @param motors The steppermotor3 class controlling the 3 deltarobot motors.
    * @return True if the calibration was succesful. False otherwise (eg. failure on sensors.)
    **/
    bool DeltaRobot::calibrateMotors(modbus_t* modbus){
        // Check the availability of the sensors
        bool sensorFailure = false;
        if(checkSensor(modbus, 0)){
            std::cout << "Sensor 0 failure (is the hardware connected?)" << std::endl;
            sensorFailure = true;
        }

        if(checkSensor(modbus, 1)){
            std::cout << "Sensor 1 failure (is the hardware connected?)" << std::endl;
            sensorFailure = true;
        }

        if(checkSensor(modbus, 2)){
            std::cout << "Sensor 2 failure (is the hardware connected?)" << std::endl;
            sensorFailure = true;
        }

        if(sensorFailure){
            return false;
        }

        // Disable limitations
        motors[0]->disableAngleLimitations();
        motors[1]->disableAngleLimitations();
        motors[2]->disableAngleLimitations();
        
        // Calibrate motors
        calibrateMotor(modbus, 0);
        calibrateMotor(modbus, 1);
        calibrateMotor(modbus, 2);

        // Set limitations
        motors[0]->setMotorLimits(Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MAX);
        motors[1]->setMotorLimits(Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MAX);
        motors[2]->setMotorLimits(Measures::MOTOR_ROT_MIN, Measures::MOTOR_ROT_MAX);
        
        // Set deviation to 0 for the 3 motors
        motors[0]->setDeviation(0);
        motors[1]->setDeviation(0);
        motors[2]->setDeviation(0);

        effectorLocation.x = 0;
        effectorLocation.y = 0;
        effectorLocation.z = -sqrt(
            ( Measures::ANKLE * Measures::ANKLE ) - ((
                Measures::BASE + Measures::HIP - Measures::EFFECTOR ) * (
                Measures::BASE + Measures::HIP - Measures::EFFECTOR )
            )
        );
        std::cout << "effector location z: " << effectorLocation.z << std::endl; 

        return true;
    }

    /**
     * Shuts down the deltarobot's hardware
     **/
    void DeltaRobot::powerOff(void)
    {
        if(motorManager->isPoweredOn())
        {
            motorManager->powerOff();
        }
    }

    /**
     * Turns on the deltarobot's hardware
     **/
    void DeltaRobot::powerOn(void)
    {
        if(!motorManager->isPoweredOn())
        {
            motorManager->powerOn();
        }
    }

    DataTypes::Point3D<double>& DeltaRobot::getEffectorLocation() {
        return effectorLocation;
    }
}
