//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           inverse_kinematics_impl.h
// Description:    inverse kinematics implementation. based on work from Viacheslav Slavinsky
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
#include <huniplacer/inverse_kinematics_model.h>

namespace huniplacer
{
	/**
	 * @brief an implementation of kinematics_model
	 **/
	class inverse_kinematics_impl : public inverse_kinematics_model
    {
        private:
            /**
             * @brief translates a point to an angle for a motor
             * @param p destination point
             * @param angle angle of the motor relative to motor 0 (in radians)
             * @return angle the motor should move to
             **/
            double moveto(const point3& p, double angle) const;
            
        public:
            inverse_kinematics_impl(
				const double base, const double hip, const double effector, const double ankle,
				const double hip_ankle_angle_max);

            virtual ~inverse_kinematics_impl(void);

            /**
             * @brief translates a point to a motion
             * @param p destination point
             * @param mf output parameter, the resulting motion is stored here
             * @return true on success, false otherwise
             **/
            void point_to_motion(const point3& p, motionf& mf) const;
    };
}
