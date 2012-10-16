/**
 * @file MotorRotation.h
 * @brief Template class for rotation data for the motor.
 * @date Created: 2012-10-03
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

namespace DataTypes{
    template<typename T>
    
    /**
     * Template class for rotation data for the motor.
     **/
    class MotorRotation{
    public:
        /**
         * @var T angle
         * The angle in radians the motor has to travel towards
         **/
        T angle;

        /**
         * @var T speed
         * The speed in degrees per second
         **/
        T speed;

        /**
         * @var T acceleration
         * The acceleration in steps per second
         **/
        T acceleration;

        /**
         * @var T deceleration
         * The deceleration in steps per second
         **/
        T deceleration;
        
        /**
         * Instantiation of rotation data for the motor, sets speed, acceleration and deceleration to default values
         **/
        MotorRotation(){
            speed = 10;
            acceleration = 360;
            deceleration = 360;
        }
        
        /**
         * Instantiation of rotation data for the motor.
         *
         * @param angle The angle in radians the motor has to travel towards
         * @param speed The speed in degrees per second
         * @param acceleration The acceleration in steps per second
         * @param deceleration The deceleration in steps per second
         **/
        MotorRotation(T angle, T speed, T acceleration, T deceleration) : angle(angle), speed(speed), acceleration(acceleration), deceleration(deceleration){}
    };
}
