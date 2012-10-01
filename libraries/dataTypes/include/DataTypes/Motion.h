//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           motion.h
// Description:    template class for motion. also 2 definitions for integer and floating point motions
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

#include <stdint.h>
#include <cstring>

namespace DataTypes
{
	/**
	 * motion.h -> Template class for use in types motioni and motionf (see below)
	 **/
    template<typename T>
    class Motion
    {
        public:
            T angles[3];
            T speed[3];
            T acceleration[3];
            T deceleration[3];
            
            Motion(bool clear = false) {
            	if(clear) {
            		memset(angles, 0, 3 * sizeof(T));
            		memset(speed, 0, 3 * sizeof(T));
            		memset(acceleration, 0, 3 * sizeof(T));
            		memset(deceleration, 0, 3 * sizeof(T));
            	}
            }
            
            Motion(
                T angle0, T angle1, T angle2,
                T speed0, T speed1, T speed2,
                T acc0, T acc1, T acc2,
                T dec0, T dec1, T dec2) {
                angles[0] = angle0;
                angles[1] = angle1;
                angles[2] = angle2;
                
                speed[0] = speed0;
                speed[1] = speed1;
                speed[2] = speed2;
                
                acceleration[0] = acc0;
                acceleration[1] = acc1;
                acceleration[2] = acc2;
                
                deceleration[0] = dec0;
                deceleration[1] = dec1;
                deceleration[2] = dec2;
            }
            
            ~Motion(void) { }
    };
    
    /**
     * Floating point motion type
     **/ 
    typedef Motion<double> MotionF;

    /** 
     * Integer motion type
     **/
    typedef Motion<uint32_t> MotionI;
}
