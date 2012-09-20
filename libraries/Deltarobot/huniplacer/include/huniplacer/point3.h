//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           point3.h
// Description:    3 dimensional point class
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

#include <cmath>

namespace huniplacer
{
	/// @brief 3 dimensional point class
    class point3
    {
        public:
            double x, y, z;

            point3(double x, double y, double z) : x(x), y(y), z(z) { }
            ~point3() { }
            
            point3 offset(point3& p);
            
            inline point3& operator+=(const point3& rhs)
            {
                x += rhs.x;
                y += rhs.y;
                z += rhs.z;
                return *this;
            }
            
            inline const point3 operator+(const point3& rhs) const
            {
                point3 res = *this;
                res += rhs;
                return res;
            }
            
            /**
             * @brief calculates the euclidean distance between *this and p
             * @return distance
             **/
            inline double distance(const point3& p) const
            {
                double dx = x - p.x;
                double dy = y - p.y;
                double dz = z - p.z;
                return sqrt(dx*dx + dy*dy + dz*dz);
            }
            
            /**
             * @brief rotate over the Y axis
             * @param phi rotation in radians
             **/
            inline point3 rotate_y(double phi) const
            {
                return point3(x*cos(phi) - z*sin(phi), y, x*sin(phi) + z*cos(phi));
            }
            
            /**
             * @brief rotate over the Z axis
             * @param phi rotation in radians
             **/
            inline point3 rotate_z(double phi) const
            {
                return point3(x*cos(phi) - y*sin(phi), x*sin(phi) + y*cos(phi), z);
            }            
    };
}
