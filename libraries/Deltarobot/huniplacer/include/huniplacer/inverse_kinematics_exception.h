//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           inverse_kinematics_exception.h
// Description:    exception thrown if an invalid value is calculated during inverse kinematics
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

#include <stdexcept>

#include <huniplacer/motion.h>
#include <huniplacer/point3.h>

namespace huniplacer
{
	/**
	 * @brief instances of deltarobot will throw this exception whenever they fail to convert a point
	 **/
    class inverse_kinematics_exception : public std::runtime_error
    {
        private:
            point3 p;
            
        public:
            inverse_kinematics_exception(const char* msg, point3 p) :
                std::runtime_error(msg),
                p(p)
            {
            }
            
            virtual ~inverse_kinematics_exception(void) throw()
            {
            }

            /**
             * @brief used to access the point that could not be converted
             **/
            point3 get_point(void)
            {
                return p;
            }
    };
}
