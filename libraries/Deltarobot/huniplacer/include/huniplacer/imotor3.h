//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           imotor3.h
// Description:    delta robot motor interface
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

#include <vector>

#include <huniplacer/motion.h>

namespace huniplacer
{
	/**
	 * imotor3.h -> Interface for the 3 motors
	 */
	class imotor3
    {
        protected:
            imotor3(void) { }
            
        public:
            virtual ~imotor3(void) { }

            /**
             * Rotate the motors
             * @param mf defines the angles, speed, acceleration and deceleration of the motors
             * @param async function is performed asyncronous if true
             */
            virtual void moveto(const motionf& mf, bool async) = 0;

            /**
             * Same as moveto, but rotates to an angle within a certain time.
             * @param time time in seconds that the motors will take to rotate to the given angle.  Speed member of given motion is ignored.
             */
            virtual void moveto_within(const motionf& mf, double time, bool async) = 0;

            /**
             * Get the minimal angle the motors can move to
             * @return angle in radians
             */
            virtual double get_min_angle(void) const = 0;

            /**
             * Get the maximum angle the motors can move to
             * @return angle in radians
             */
            virtual double get_max_angle(void) const = 0;

            /**
             * Stops the motors
             */
            virtual void stop(void) = 0;

            /**
             * Wait for all motor controllers to become idle
             * @param timeout time in milliseconds until timeout (0 means infinite)
             * @return false if timed out, true otherwise
             */
            virtual bool wait_for_idle(long timeout) = 0;

            /**
             * Test whether the motor controllers are idle
             * @return true if idle, false otherwise
             */
            virtual bool is_idle(void) = 0;

            /**
             * Shuts down the motors
             */
            virtual void power_off(void) = 0;

            /**
             * @brief turns on the motors
             */
            virtual void power_on(void) = 0;

            /**
             * Determine if the motor driver(s) are powered on
             * @return true of powered on, false otherwise
             */
            virtual bool is_powerd_on(void) = 0;

            /**
             * Synchronises software and real world angels.
             * @param angles the new angles
             */
            virtual void override_current_angles(double * angles) = 0;
    };
}
