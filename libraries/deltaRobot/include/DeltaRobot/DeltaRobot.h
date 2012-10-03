//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           deltarobot.h
// Description:    symbolizes an entire deltarobot
// Author:         Lukas Vermond & Kasper van Nieuwland
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

#include <modbus/modbus.h>
#include <DataTypes/Point3D.h>
#include <DataTypes/DeltaRobotMeasures.h>
#include <Motor/MotorInterface.h>
#include <Motor/StepperMotor.h>
#include <Motor/MotorManager.h>
#include <DeltaRobot/EffectorBoundaries.h>

namespace DeltaRobot
{
	class InverseKinematicsModel;

	
    class DeltaRobot 
    {
        public:
            DeltaRobot(DataTypes::DeltaRobotMeasures& drm, Motor::MotorManager* motorManager, Motor::StepperMotor* (&motors)[3]);
            ~DeltaRobot();
            
            inline EffectorBoundaries* getBoundaries() { return boundaries; }
            inline bool hasBoundaries() { return boundariesGenerated; }

            void generateBoundaries(double voxelSize);
            bool checkPath(const DataTypes::Point3D<double>& begin, const DataTypes::Point3D<double>& end);
            void moveTo(const DataTypes::Point3D<double>& p, double speed);
            void calibrateMotor(modbus_t* modbus, int motorIndex);
            bool checkSensor(modbus_t* modbus, int sensorIndex);
            bool calibrateMotors(modbus_t* modbus);
            void powerOff();
            void powerOn();
            DataTypes::Point3D<double>& getEffectorLocation();

        private:
            InverseKinematicsModel* kinematics;
            Motor::StepperMotor* (&motors)[3];
            Motor::MotorManager* motorManager;
            EffectorBoundaries* boundaries;

            DataTypes::Point3D<double> effectorLocation;
            bool boundariesGenerated;

            bool isValidAngle(int motorIndex, double angle);
    };
}
