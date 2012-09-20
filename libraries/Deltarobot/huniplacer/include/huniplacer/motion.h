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
// License:        GNU GPL v3
//
// This file is part of huniplacer.
//
// huniplacer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// huniplacer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with huniplacer.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#pragma once

#include <stdint.h>
#include <cstring>

namespace huniplacer
{
	/**
	 * @brief template class for use in types motioni and motionf (see below)
	 **/
    template<typename T>
    class motion
    {
        public:
            T angles[3];
            T speed[3];
            T acceleration[3];
            T deceleration[3];
            
            motion(bool clear = false)
            {
            	if(clear)
            	{
            		memset(angles, 0, 3 * sizeof(T));
            		memset(speed, 0, 3 * sizeof(T));
            		memset(acceleration, 0, 3 * sizeof(T));
            		memset(deceleration, 0, 3 * sizeof(T));
            	}
            }
            
            motion(
                T angle0, T angle1, T angle2,
                T speed0, T speed1, T speed2,
                T acc0, T acc1, T acc2,
                T dec0, T dec1, T dec2)
            {
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
            
            ~motion(void) { }
    };
    
    /// @brief floating point motion type
    typedef motion<double> motionf;

    /// @brief integer motion type
    typedef motion<uint32_t> motioni;
}
