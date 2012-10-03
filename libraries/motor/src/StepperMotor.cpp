//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        steppermotor3.cpp
// File:           steppermotor driver
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
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


#include <Motor/StepperMotor.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <Utilities/Utilities.h>
#include <Motor/CRD514KD.h>
#include <Motor/CRD514KDException.h>
#include <Motor/MotorException.h>

/** 
 * steppermotor3.cpp -> Implementation of imotor3 for steppermotors
 **/

namespace Motor
{



    StepperMotor::~StepperMotor(void)
    {

        try {
            powerOff();
        } catch(std::runtime_error& err) {}
    }

    void StepperMotor::powerOn(void)
    {
        if(!poweredOn) {
            boost::lock_guard<boost::mutex> lock(modbus->modbusMutex);
            //reset alarm
            modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 0);
            //set operating modes
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_POSMODE, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_OPMODE, 0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SEQ_MODE + 0, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SEQ_MODE + 1, /*1*/
            0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
            //set motors limits
            modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((maxAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
            modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((minAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
            modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_START_SPEED, 1);
            //clear counter
            modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 0);

            currentAngle = 0;
            poweredOn = true;
        }
    }

    void StepperMotor::powerOff(void)
    {
        if(poweredOn) {
            stop();
            boost::lock_guard<boost::mutex> lock(modbus->modbusMutex);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);
            poweredOn = false;
        }
    }

    /**
     * Stops the motors & clears the motion queue
     * @note (un)locks queue_mutex
     **/
    void StepperMotor::stop(void)
    {
        if(!poweredOn)
        {
            throw MotorException("motor drivers are not powered on");
        }
        
        try
        {
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::STOP);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
        }
        catch(ModbusController::ModbusException& ex)
        {
            std::cerr << "steppermotor::stop failed: " << std::endl << "what(): " << ex.what() << std::endl;
        }
    }

  void StepperMotor::resetCounter(){

        waitTillReady();
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);

        //clear counter
        modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 1);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 0);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
    }

    void StepperMotor::setMotorLimits(double minAngle, double maxAngle){
        //set motors limits
        setMinAngle(minAngle);
        setMaxAngle(maxAngle);
    }

    
    void StepperMotor::moveTo(const DataTypes::MotorRotation<double>& mr){
        writeRotationData(mr);
        startMovement();
    }

    /**
     * @brief Writes the rotation data into the motor controller
     * @param mr a MotorRotation
     **/
    void StepperMotor::writeRotationData(const DataTypes::MotorRotation<double>& mr, bool breaking)
    {
        std::cout << "writeRotationData speed" << mr.speed << std::endl;
        if(!poweredOn)
        {
        	throw MotorException("motor drivers are not powered on");
        }

        if(anglesLimited && (mr.angle <= minAngle || mr.angle >= maxAngle)){
            throw std::out_of_range("one or more angles out of range");
        }

        uint32_t motorSteps = (uint32_t)(mr.angle / CRD514KD::MOTOR_STEP_ANGLE);
        uint32_t motorSpeed = (uint32_t)(mr.speed / CRD514KD::MOTOR_STEP_ANGLE);
        uint32_t motorAcceleration = (uint32_t)(CRD514KD::MOTOR_STEP_ANGLE * 1000000000.0 / mr.acceleration);
        uint32_t motorDeceleration = (uint32_t)(CRD514KD::MOTOR_STEP_ANGLE * 1000000000.0 / mr.deceleration);
char k;
        if(breaking){
        std::cout << "Writing data " << motorIndex << "\t" << minAngle << "\t" << maxAngle << std::endl;
        
        std::cin >> k;
        }
        std::cout << "Speed " << motorSpeed << "\t" << mr.speed << "\t" << CRD514KD::MOTOR_STEP_ANGLE << std::endl;
        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_SPEED, motorSpeed, true);
        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_POS, motorSteps, true);
        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_ACC, motorAcceleration, true);
        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_DEC, motorDeceleration, true);
    }

    /**
    * @brief Start the movement of the motor
    */
    void StepperMotor::startMovement(){
        if(!poweredOn)
        {
            throw MotorException("motor drivers are not powered on");
        }

        //execute motion
        waitTillReady();

        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON | CRD514KD::CMD1Bits::START);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
    }


    /**
     * Same as moveto, but rotates to an angle within a certain time.
     * @param time time in seconds that the motors will take to rotate to the given angle. Speed member of given motion is ignored
     **/
    void StepperMotor::moveToWithin(const DataTypes::MotorRotation<double>& mr, double time, bool start)
    {
        DataTypes::MotorRotation<double> newmr = mr;
        std::cout << "moveToWithin old speed" << mr.speed << " " << newmr.speed << std::endl;
        std::cout << "moveToWithin calc: cur angle " << currentAngle << " new angle " << mr.angle << " time " << time << std::endl;
        newmr.speed = fabs(currentAngle - mr.angle) / time;
        std::cout << "moveToWithin new speed" << mr.speed << " " << newmr.speed << std::endl;

        if(start){
            moveTo(newmr);
        } else {
            writeRotationData(newmr, true);
        }
    }

    void StepperMotor::waitTillReady(void)
    {
       uint16_t status_1;
       while(!((status_1 = modbus->readU16(motorIndex, CRD514KD::Registers::STATUS_1)) & CRD514KD::Status1Bits::READY))
       {
            if((status_1 & CRD514KD::Status1Bits::ALARM) || (status_1 & CRD514KD::Status1Bits::WARNING)) {
                std::cout << "Motor: " << motorIndex << " Alarm code: " << std::hex << modbus->readU16(motorIndex, 0x100) << "h" << std::endl;
                
                throw CRD514KDException(
                    motorIndex, status_1 & CRD514KD::Status1Bits::WARNING,
                    status_1 & CRD514KD::Status1Bits::ALARM);
            }
        }
    }

    void StepperMotor::setMinAngle(double minAngle)
    {
    	this->minAngle = minAngle;
        modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((minAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
        anglesLimited = true;
    }

    void StepperMotor::setMaxAngle(double maxAngle)
    {
    	this->maxAngle = maxAngle;
        modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((maxAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
        anglesLimited = true;
    }

    void StepperMotor::disableAngleLimitations() {
        modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SOFTWARE_OVERTRAVEL, 0);
        anglesLimited = false;
    }
}
