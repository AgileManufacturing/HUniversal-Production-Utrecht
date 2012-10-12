/**
 * @file StepperMotor.cpp
 * @brief Steppermotor driver.
 * @date Created: 2012-10-02
 *
 * @author Koen Braham
 * @author Dennis Koole
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

#include <Motor/StepperMotor.h>

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include <Utilities/Utilities.h>
#include <Motor/CRD514KD.h>
#include <Motor/CRD514KDException.h>
#include <Motor/MotorException.h>

namespace Motor{
    StepperMotor::StepperMotor(ModbusController::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex) :
        MotorInterface(), modbus(modbusController), motorIndex(motorIndex), anglesLimited(false), poweredOn(false)  {}

    StepperMotor::StepperMotor(ModbusController::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex, double minAngle, double maxAngle):
        MotorInterface(), minAngle(minAngle), maxAngle(maxAngle), modbus(modbusController), motorIndex(motorIndex), anglesLimited(true), poweredOn(false){}

    StepperMotor::~StepperMotor(void){
        try{
            powerOff();
        } catch(std::runtime_error& err){}
    }

    // Need comment!
    void StepperMotor::powerOn(void){
        if(!poweredOn){
            boost::lock_guard<boost::mutex> lock(modbus->modbusMutex);

            //Reset alarm
            modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::RESET_ALARM, 0);

            // Set operating modes
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_POSMODE, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_OPMODE, 0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SEQ_MODE + 0, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SEQ_MODE + 1, /*1*/ 0);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
            
            // Set motor limits
            modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((maxAngle - deviation) / CRD514KD::MOTOR_STEP_ANGLE));
            modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((minAngle - deviation) / CRD514KD::MOTOR_STEP_ANGLE));
            modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_START_SPEED, 1);
            
            // Clear counter
            modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 1);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 0);

            currentAngle = 0;
            poweredOn = true;
        }
    }

    // Need comment!
    void StepperMotor::powerOff(void){
        if(poweredOn){
            stop();
            boost::lock_guard<boost::mutex> lock(modbus->modbusMutex);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);
            poweredOn = false;
        }
    }

    /**
     * @brief Stops the motors & clears the motion queue.
     * 
     * @note (un)locks queue_mutex.
     **/
    void StepperMotor::stop(void){
        if(!poweredOn){
            throw MotorException("motor drivers are not powered on");
        }
        
        try{
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::STOP);
            modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
        } catch(ModbusController::ModbusException& exception){
            std::cerr << "steppermotor::stop failed: " << std::endl << "what(): " << exception.what() << std::endl;
        }
    }

    // Need comment!
    void StepperMotor::resetCounter(void){
        waitTillReady();
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, 0);

        // Clear counter.
        modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 1);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CLEAR_COUNTER, 0);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
    }

    // Need comment!
    void StepperMotor::setMotorLimits(double minAngle, double maxAngle){
        // Set motors limits.
        setMinAngle(minAngle);
        setMaxAngle(maxAngle);
    }

    // Need comment!
    void StepperMotor::moveTo(const DataTypes::MotorRotation<double>& motorRotation){
        writeRotationData(motorRotation);
        startMovement();
    }

    /**
     * @brief Writes the rotation data into the motor controller.
     * 
     * @param motorRotation A MotorRotation.
     **/
    void StepperMotor::writeRotationData(const DataTypes::MotorRotation<double>& motorRotation){
        if(!poweredOn){
        	throw MotorException("motor drivers are not powered on");
        }

        if(anglesLimited && (motorRotation.angle <= minAngle || motorRotation.angle >= maxAngle)){
            throw std::out_of_range("one or more angles out of range");
        }

        uint32_t motorSteps = (uint32_t)((motorRotation.angle + deviation) / CRD514KD::MOTOR_STEP_ANGLE);
        uint32_t motorSpeed = (uint32_t)(motorRotation.speed / CRD514KD::MOTOR_STEP_ANGLE);
        uint32_t motorAcceleration = (uint32_t)(CRD514KD::MOTOR_STEP_ANGLE * 1000000000.0 / motorRotation.acceleration);
        uint32_t motorDeceleration = (uint32_t)(CRD514KD::MOTOR_STEP_ANGLE * 1000000000.0 / motorRotation.deceleration);

        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_SPEED, motorSpeed, true);
        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_POS, motorSteps, true);
        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_ACC, motorAcceleration, true);
        modbus->writeU32(motorIndex, CRD514KD::Registers::OP_DEC, motorDeceleration, true);
        setAngle = motorRotation.angle;
    }

    /**
     * @brief Start the movement of the motor.
     **/
    void StepperMotor::startMovement(void){
        if(!poweredOn){
            throw MotorException("motor drivers are not powered on");
        }

        // Execute motion.
        waitTillReady();

        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON | CRD514KD::CMD1Bits::START);
        modbus->writeU16(motorIndex, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);
        updateAngle();
    }


    /**
     * @brief Same as moveto, but rotates to an angle within a certain time.
     * 
     * @param time Time in seconds that the motors will take to rotate to the given angle. Speed member of given motion is ignored.
     **/
    void StepperMotor::moveToWithin(const DataTypes::MotorRotation<double>& motorRotation, double time, bool start){
        DataTypes::MotorRotation<double> newMotorRotation = motorRotation;
        newMotorRotation.speed = fabs(currentAngle - motorRotation.angle) / time;
        if(start){
            moveTo(newMotorRotation);
        } else{
            writeRotationData(newMotorRotation);
        }
    }

    // Need comment!
    void StepperMotor::waitTillReady(void){
       uint16_t status_1;
       while(!((status_1 = modbus->readU16(motorIndex, CRD514KD::Registers::STATUS_1)) & CRD514KD::Status1Bits::READY)){
            if((status_1 & CRD514KD::Status1Bits::ALARM) || (status_1 & CRD514KD::Status1Bits::WARNING)){
                std::cerr << "Motor: " << motorIndex << " Alarm code: " << std::hex << modbus->readU16(motorIndex, CRD514KD::Registers::PRESENT_ALARM) << "h" << std::endl;
                
                throw CRD514KDException(
                    motorIndex, status_1 & CRD514KD::Status1Bits::WARNING,
                    status_1 & CRD514KD::Status1Bits::ALARM);
            }
        }
    }

    // Need comment!
    void StepperMotor::setMinAngle(double minAngle){
    	this->minAngle = minAngle;
        modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_NEGATIVE, (uint32_t)((minAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
        anglesLimited = true;
    }

    // Need comment!
    void StepperMotor::setMaxAngle(double maxAngle){
    	this->maxAngle = maxAngle;
        modbus->writeU32(motorIndex, CRD514KD::Registers::CFG_POSLIMIT_POSITIVE, (uint32_t)((maxAngle + deviation) / CRD514KD::MOTOR_STEP_ANGLE));
        anglesLimited = true;
    }

    // Need comment!
    void StepperMotor::disableAngleLimitations(void){
        modbus->writeU16(motorIndex, CRD514KD::Registers::OP_SOFTWARE_OVERTRAVEL, 0);
        anglesLimited = false;
    }

    // Need comment!
    void StepperMotor::updateAngle(void){
        currentAngle = setAngle;
    }
}
