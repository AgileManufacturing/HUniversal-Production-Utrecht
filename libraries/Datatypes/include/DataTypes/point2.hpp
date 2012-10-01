//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        datatypes
// File:           point2.hpp
// Description:    A object used for storing a coordinated as 2 floats
// Author:         Kasper van Nieuwland & Zep Mouris
// Notes:          ...
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
#include <cstdlib>
#include <vector>

namespace datatypes {
	/**
	* class that represents a 2 dimensional point.
	* Has utilities to transform the point.
	*/
	template<typename T>
	class point2 {
		public:
			/**
			 * contains the x value of the point
			 */
			T x;
			/**
			 * contains the y value of the point
			 */
			T y;
			/**
			 * default constructor
			 */
			point2(void) {}
			/**
			 * constructor to make a point at a specific location
			 * @param x the x value of the point
			 * @param y the y value of the point
			 */
			point2(T x, T y) : x(x), y(y) { }
			/**
			 * destructor
			 */
			~point2() { }

            inline point2<T>& operator+=(const point2<T>& rhs)
            {
                x += rhs.x;
                y += rhs.y;
                return *this;
            }

            inline point2<T>& operator-=(const point2<T>& rhs)
            {
                x -= rhs.x;
                y -= rhs.y;
                return *this;
            }

            inline point2<T>& operator*=(const point2<T>& rhs)
            {
                x *= rhs.x;
                y *= rhs.y;
                return *this;
            }

            
            /**
             * calculates the distance between two points
             * @param p the second point
             * @return the distance between the points
             */
            inline double distance(const point2<T>& p) const
            {
                double dx = x - p.x;
                double dy = y - p.y;
                return sqrt(dx*dx + dy*dy);
            }
            /**
             * Calculate the center of two points
             * @param p the second point
             * @return returns a point2f with the location of the center
             */
            inline point2<T> mean(const point2<T>& p) const
			{
            	point2<T> result;
            	result = *this+p;
            	result.x /= 2;
            	result.y /= 2;
				return result;
			}
            /**
             * rotates the point around 0,0
             * @param angle the angle to rotate (radians)
             * @return returns a point2f with the new location
             */
            inline point2<T> rotate(double angle) const
            {
            	return point2<T>(x*cos(angle) - y*sin(angle), x*sin(angle) + y*cos(angle));
            }
	};
	
	template<typename T>
	bool operator==(const point2<T>& lhs, const point2<T>& rhs)
	{
		return lhs.x == rhs.x && lhs.y == rhs.y;
	}
	
	template<typename T>
	point2<T> operator+(const point2<T>& lhs, const point2<T>& rhs)
    {
        point2<T> res = lhs;
        res += rhs;
        return res;
    }

    template<typename T>
	point2<T> operator-(const point2<T>& lhs, const point2<T>& rhs)
    {
        point2<T> res = lhs;
        res -= rhs;
        return res;
    }

    template<typename T>
	point2<T> operator*(const point2<T>& lhs, const point2<T>& rhs)
    {
        point2<T> res = lhs;
        res *= rhs;
        return res;
    }
	
	typedef point2<float> point2f;
	typedef point2<double> point2lf;
	typedef point2<int> point2i;
}
