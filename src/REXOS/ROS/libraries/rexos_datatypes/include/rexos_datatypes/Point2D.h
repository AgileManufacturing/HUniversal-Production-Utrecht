/**
 * @file Point2D.h
 * @brief A object used for storing a coordinated as 2 doubles and useful math methods.
 * @date Created: 2011-11-11
 *
 * @author Kasper van Nieuwland
 * @author Zep Mouris
 *
 * @section LICENSE
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/
#pragma once

#include <cmath>
#include <vector>

namespace rexos_datatypes{
	/**
	 * Class that represents a 2 dimensional point. Has utilities to transform the point.
	 **/
	class Point2D{
	public:
		/**
		 * @var double x
		 * Contains the x value of the point.
		 **/
		double x;
		/**
		 * @var double y
		 * Contains the y value of the point.
		 **/
		double y;

		/**
		 * Constructor to make a point to (0,0).
		 **/
		Point2D( ) : x(0), y(0){}

		/**
		 * Constructor to make a point at a specific location.
		 * 
		 * @param x the x value of the point
		 * @param y the y value of the point
		 **/
		Point2D(double x, double y) : x(x), y(y){}

		/**
		 * Destructor.
		 **/
		~Point2D(){}

		/**
		 * Compare operator for Point 2D
		 **/
		inline bool operator==(const Point2D& rhs) const{
			return this->x == rhs.x && this->y == rhs.y;
		}

		/** 
		 * Addition assignment
		 **/
		inline Point2D& operator+=(const Point2D& rhs){
			x += rhs.x;
			y += rhs.y;
			return *this;
		}

		/**
		 * Substract assignment
		 **/
		inline Point2D& operator-=(const Point2D& rhs){
			x -= rhs.x;
			y -= rhs.y;
			return *this;
		}

		/**
		 * Multiplication assignment
		 **/
		inline Point2D& operator*=(const Point2D& rhs){
			x *= rhs.x;
			y *= rhs.y;
			return *this;
		}

		/** 
		 * Addition of points
		 **/
		inline Point2D operator+(const Point2D& rhs) const{
			Point2D res = *this;
			res += rhs;
			return res;
		}

		/**
		 * Substract of points
		 **/
		inline Point2D operator-(const Point2D& rhs) const{
			Point2D res = *this;
			res -= rhs;
			return res;
		}

		/**
		 * Multiplication operator
		 **/
		inline Point2D operator*(const Point2D& rhs) const{
			Point2D res = *this;
			res *= rhs;
			return res;
		}

		/**
		 * Calculates the distance between this point and another points
		 *
		 * @param point The other point.
		 *
		 * @return The distance between the points.
		 **/
		inline double distance(const Point2D& point) const{
			double deltaX = x - point.x;
			double deltaY = y - point.y;
			return sqrt(deltaX * deltaX + deltaY * deltaY);
		}

		/**
		 * Calculate the center between this point and another points.
		 *
		 * @param point The other point.
		 *
		 * @return Returns a Point2D with the location of the center.
		 **/
		inline Point2D mean(const Point2D& point) const{
			Point2D result;
			result = *this + point;
			result.x /= 2;
			result.y /= 2;
			return result;
		}

		/**
		 * Rotates the point around 0,0
		 *
		 * @param angle The angle to rotate in radians.
		 *
		 * @return Returns a Point2D with the new location.
		 **/
		inline Point2D rotate(double angle) const{
			return Point2D(x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle));
		}
	};
}
