//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        datatypes
// File:           point2f.cpp
// Description:    A object used for storing a coordinated as 2 floats
// Author:         Kasper van Nieuwland & Zep Mouris
// Notes:          ...
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
