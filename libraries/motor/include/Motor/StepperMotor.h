/**
 * @file StepperMotor.h
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

#pragma once

#include <queue>
#include <boost/thread.hpp>

#include <DataTypes/MotorRotation.h>
#include <ModbusController/ModbusException.h>
#include <ModbusController/ModbusController.h>
#include <Motor/CRD514KD.h>
#include <Motor/MotorInterface.h>

namespace Motor{
    /**
     * Steppermotor driver.
     **/
    class StepperMotor : public MotorInterface{
    public:
        StepperMotor(ModbusController::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex, double minAngle, double maxAngle);

        virtual ~StepperMotor(void);
    
        void powerOn(void);
        void powerOff(void);
        void stop(void);

        void resetCounter(void);
        void setMotorLimits(double minAngle, double maxAngle);

        void moveTo(const DataTypes::MotorRotation& motorRotation);
        void writeRotationData(const DataTypes::MotorRotation& motorRotation, bool useDeviation = true);

        void startMovement(void);
        void moveToWithin(const DataTypes::MotorRotation& motorRotation, double time, bool start);
        void waitTillReady(void);

        bool isPoweredOn(void){ return poweredOn; }

        /**
         * Returns the minimum angle, in radians, the StepperMotor can travel on the theoretical plane.
         * 
         * @return The minimum angle, in radians, the StepperMotor can travel on the theoretical plane.
         **/
        inline double getMinAngle(void) const{ return minAngle; }
        
        /**
         * Returns the maximum angle, in radians, the StepperMotor can travel on the theoretical plane.
         * 
         * @return The maximum angle, in radians, the StepperMotor can travel on the theoretical plane. 
         **/
        inline double getMaxAngle(void) const{ return maxAngle; }
        void setMinAngle(double minAngle);
        void setMaxAngle(double maxAngle);
        
        /**
         * Stores the angle that was given to the motor in the local variable currentAngle.
         *
         * @param angle The angle that is the current location of the motor.
         **/
        void setCurrentAngle(double angle){ currentAngle = angle; }

        /**
         * Returns the deviation between the motors 0 degrees and the horizontal 0 degrees.
         *
         * @return The deviation between the hardware and theoretical 0 degrees.
         **/
        double getDeviation(void){ return deviation; }

        /**
         * Sets the deviation between the motors 0 degrees and the horizontal 0 degrees.
         *
         * @param deviation The deviation between the hardware and theoretical 0 degrees.
         **/
        void setDeviation(double deviation){ this->deviation = deviation; }

        void disableAngleLimitations(void);
        void updateAngle(void);

    private:
        /**
         * @var double currentAngle
         * The angle most recently written to the motors since the most recently executed movement.
         **/
        double currentAngle;

        /**
         * @var double setAngle
         * The angle most recently written to the motors. This does not mean the movement is executed.
         **/
        double setAngle;

        /**
         * @var double deviation
         * The deviation between the motors 0 degrees and the horizontal 0 degrees.
         **/
        double deviation;

        /**
         * @var double minAngle
         * The minimum for the angle, in radians, the StepperMotor can travel on the theoretical plane.
         **/
        double minAngle;

        /**
         * @var double maxAngle
         * The maximum angle, in radians, the StepperMotor can travel on the theoretical plane.
         **/
        double maxAngle;
        
        /**
         * @var ModbusController* modbus
         * Controller for the modbus communication.
         **/
        ModbusController::ModbusController* modbus;

        /**
         * @var Slaves::t motorIndex
         * Index for the motor in the CRD514KD slaves enum.
         **/
        CRD514KD::Slaves::t motorIndex;
        
        /**
         * @var bool anglesLimited
         * If hardware limitations are set on the angles the motor can travel.
         **/
        bool anglesLimited;

        /**
         * @var volatile bool poweredOn
         * If the motor is already powered on.
         **/
        volatile bool poweredOn;
    };
}
