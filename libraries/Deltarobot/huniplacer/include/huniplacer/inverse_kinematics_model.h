//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           inverse_kinematics_model.h
// Description:    abstract inverse kinematics model for use in deltarobot class
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

#include <huniplacer/point3.h>
#include <huniplacer/motion.h>

namespace huniplacer
{
	/**
	 * @brief kinematics model of the deltarobot
	 *
	 * uses various lengths and sizes to calculate a point to a motion
	 *
	 *  conventions:
     *	sitting in front of delta robot
     *	x-axis goes from left to right
     *	y-axis goes from front to back
     *	z-axis goes from bottom to top
     *	point (0,0,0) lies in the middle of all the motors at the motor's height
	 **/
    class inverse_kinematics_model
    {
        protected:
    		/// @brief radius of the base in millimeters
            const double base;

            /// @brief length of the hip in millimeters
            const double hip;

            /// @brief radius of the effector in millimeters
            const double effector;

            /// @brief length of the ankle in millimeters
            const double ankle;
        
            inverse_kinematics_model(
				const double base, const double hip, const double effector, const double ankle,
				const double hip_ankle_angle_max) :
                base(base),
                hip(hip),
                effector(effector),
                ankle(ankle),
                hip_ankle_angle_max(hip_ankle_angle_max)
            { }
            
        public:
            virtual ~inverse_kinematics_model(void) { }

            /// @brief maximum angle between hip and angle on x-z plane in radians
            const double hip_ankle_angle_max;
            
            /**
             * @brief converts a point to a motion
             * @param p point that shall be converted
             * @param mf output parameter, the results of the conversion will be stored here
             **/
            virtual void point_to_motion(const point3& p, motionf& mf) const = 0;
    };
}
