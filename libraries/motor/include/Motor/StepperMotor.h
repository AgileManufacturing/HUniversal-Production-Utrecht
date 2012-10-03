//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           Steppermotor.h
// Description:    steppermotor driver
// Author:         Koen Braham & Dennis Koole
// Notes:          -
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


#pragma once

#include <queue>
#include <boost/thread.hpp>

#include <DataTypes/MotorRotation.h>
#include <ModbusController/ModbusException.h>
#include <ModbusController/ModbusController.h>
#include <Motor/CRD514KD.h>
#include <Motor/MotorInterface.h>

namespace Motor
{
    class StepperMotor : public MotorInterface
    {
        public:
            //StepperMotor(ModbusController::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex);
            //StepperMotor(ModbusController::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex, double minAngle, double maxAngle);

                StepperMotor(ModbusController::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex) :
        MotorInterface(), modbus(modbusController), motorIndex(motorIndex), anglesLimited(false), poweredOn(false)  {}

    StepperMotor(ModbusController::ModbusController* modbusController, CRD514KD::Slaves::t motorIndex, double minAngle, double maxAngle):
        MotorInterface(), minAngle(minAngle), maxAngle(maxAngle), modbus(modbusController), motorIndex(motorIndex), anglesLimited(true), poweredOn(false)  {}

            virtual ~StepperMotor();
        
            void powerOn();
            void powerOff();
            void stop();

            void resetCounter();
            void setMotorLimits(double minAngle, double maxAngle);


            void moveTo(const DataTypes::MotorRotation<double>& mr);
            /**
             * Set the rotate data for the motors
             * @param mf defines the angles, speed, acceleration and deceleration of the motors
             * @param async function is performed asyncronous if true
             **/
            void writeRotationData(const DataTypes::MotorRotation<double>& mr, bool breaking = false);

            /**
             * Start the rotation of the motors
             **/
            void startMovement();
            void moveToWithin(const DataTypes::MotorRotation<double>& mr, double time, bool start);
            void waitTillReady();

            bool isPoweredOn() { return poweredOn; }
            inline double getMinAngle() const { return minAngle; }
            inline double getMaxAngle() const { return maxAngle; }
            void setMinAngle(double minAngle);
            void setMaxAngle(double maxAngle);

            double getDeviation(){ return deviation; }
            void setDeviation(double deviation){ deviation = deviation; }

            void disableAngleLimitations();
        private:
            double currentAngle, deviation, minAngle, maxAngle;
            
            ModbusController::ModbusController* modbus;

            CRD514KD::Slaves::t motorIndex;
            
            bool anglesLimited;
            volatile bool poweredOn;
    };
}
