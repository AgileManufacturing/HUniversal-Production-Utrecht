//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        datatypes
// File:           point3.hpp
// Description:    3 dimensional point class
// Author:         Lukas Vermond
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
