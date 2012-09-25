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
#include <huniplacer/Point3D.h>
#include <huniplacer/imotor3.h>
#include <huniplacer/effector_boundaries.h>
#include <huniplacer/huniplacer.h>

//TODO: implement forward kinematics and use that to calculate the current effector position

/**
 * @brief holds huniplacer related classes
 **/
namespace huniplacer
{
	class InverseKinematicsModel;

	/**
	 * @brief this class symbolises an entire deltarobot
	 **/
    class deltarobot 
    {
        private:
            InverseKinematicsModel& kinematics;
            steppermotor3& motors;
            effector_boundaries* boundaries;

            Point3D effector_location;
            bool boundaries_generated;

            bool is_valid_angle(double angle);
        
        public:
            /**
             * @brief constructor
             * @param kinematics kinematics model that will be used to convert points to motions
             * @param motors implementation of motor interface that will be used to communicate with the motors
             **/
            deltarobot(InverseKinematicsModel& kinematics, steppermotor3& motors);

            ~deltarobot(void);
            
            inline effector_boundaries* get_boundaries();
            inline bool has_boundaries();
            void generate_boundaries(double voxel_size);

            /**
			 * @brief checks the path between two points
			 * @param begin start point
			 * @param end finish point
			 **/
            bool check_path(const Point3D& begin,const Point3D& end);

            /**
             * @brief makes the deltarobot move to a point
             * @param p 3-dimensional point to move to
             * @param speed movement speed speed in degrees per second
             * @param async motions will be stored in a queue for later execution if true
             **/
            void moveto(const Point3D& p, double speed, bool async = true);

            void calibrateMotor(modbus_t* modbus, int motorIndex);
            bool checkSensor(modbus_t* modbus, int sensorIndex);
            void calibrateMotors(modbus_t* modbus);

            /**
             * @brief stops the motors
             **/
            void stop(void);

            /**
             * @brief wait for the deltarobot to become idle
             *
             * the deltarobot is idle when all it's motions are completed
             * and it has stopped moving
             *
             * @param timeout time in milliseconds for the wait to timeout. 0 means infinite
             **/
            bool wait_for_idle(long timeout = 0);

            /**
             * @brief true if the deltarobot is idle, false otherwise
             **/
            bool is_idle(void);

            /**
             * @brief shuts down the deltarobot's hardware
             */
            void power_off(void);

            /**
             * @brief turns on the deltarobot's hardware
             */
            void power_on(void);

            Point3D& getEffectorLocation();
    };

    effector_boundaries* deltarobot::get_boundaries(){return boundaries;}
    bool deltarobot::has_boundaries(){return boundaries_generated;}
}
