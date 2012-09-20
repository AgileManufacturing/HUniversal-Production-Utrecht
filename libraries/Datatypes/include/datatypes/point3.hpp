//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        datatypes
// File:           point3.h
// Description:    3 dimensional point class
// Author:         Lukas Vermond
// Notes:          -
//
// License:        GNU GPL v3
//
// This file is part of datatypes.
//
// datatypes is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// datatypes is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with datatypes.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#pragma once

#include <cmath>

namespace datatypes
{
	/// @brief 3 dimensional point class
	template<typename T>
    class point3
    {
        public:
            T x, y, z;

            point3(void) { }
            point3(T x, T y, T z) : x(x), y(y), z(z) { }
            ~point3(void) { }
            
            point3 offset(point3<T>& p);
            
            point3<T>& operator+=(const point3<T>& rhs)
            {
                x += rhs.x;
                y += rhs.y;
                z += rhs.z;
                return *this;
            }
            
            /**
             * @brief calculates the euclidean distance between *this and p
             * @return distance
             **/
            T distance(const point3<T>& p) const
            {
                T dx = x - p.x;
                T dy = y - p.y;
                T dz = z - p.z;
                return sqrt(dx*dx + dy*dy + dz*dz);
            }
            
            /**
             * @brief rotate over the Y axis
             * @param phi rotation in radians
             **/
            point3<T> rotate_y(T phi) const
            {
                return point3<T>(x*cos(phi) - z*sin(phi), y, x*sin(phi) + z*cos(phi));
            }
            
            /**
             * @brief rotate over the Z axis
             * @param phi rotation in radians
             **/
            point3<T> rotate_z(T phi) const
            {
                return point3<T>(x*cos(phi) - y*sin(phi), x*sin(phi) + y*cos(phi), z);
            }            
    };
	
	template<typename T>
	point3<T> operator+(const point3<T>& lhs, const point3<T>& rhs)
    {
        point3<T> res = lhs;
        res += rhs;
        return res;
    }
	
	typedef point3<float> point3f;
	typedef point3<double> point3lf;
	typedef point3<int> point3i;
}
