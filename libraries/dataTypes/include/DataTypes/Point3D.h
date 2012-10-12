/**
 * @file Point3D.h
 * @brief 3 dimensional point class.
 * @date Created: 2012-10-03
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Dennis Koole
 * @author 1.1 Daan Veltman
 *
 * @section LICENSE
 * License: newBSD
 *
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
 * @note 2012-10-11 DV: Removed unused methods
 **/

#pragma once

#include <cmath>

namespace DataTypes{
	template<typename T>

	class Point3D{
	public:
		T x, y, z;

		Point3D(){}
		Point3D(T x, T y, T z) : x(x), y(y), z(z){}

		inline Point3D<T>& operator+=(const Point3D<T>& rhs){
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
			return *this;
		}

		inline const Point3D<T> operator+(const Point3D<T>& rhs) const{
			Point3D<T> result = *this;
			result += rhs;
			return result;
		}

		/**
		 * Calculates the euclidean distance between *this and p.
		 *
		 * @param p The point to calculate the euclidean distance to from *this.
		 * 
		 * @return the euclidean distance.
		 **/
		inline T distance(const Point3D<T>& point) const{
			T dx = x - point.x;
			T dy = y - point.y;
			T dz = z - point.z;
			return sqrt(dx * dx + dy * dy + dz * dz);
		}

		/**
		 * Rotate the point around the Y axis.
		 *
		 * @param rotationRadians Rotation in radians.
		 * 
		 * @return the rotated point.
		 **/
		inline Point3D<T> rotateAroundYAxis(T rotationRadians) const{
			return Point3D<T>(x * cos(rotationRadians) - z * sin(rotationRadians), y,
					x * sin(rotationRadians) + z * cos(rotationRadians));
		}

		/**
		 * Rotate the point around the Z axis.
		 *
		 * @param rotationRadians Rotation in radians.
		 * 
		 * @return the rotated point.
		 **/
		inline Point3D<T> rotateAroundZAxis(T rotationRadians) const{
			return Point3D<T>(x * cos(rotationRadians) - y * sin(rotationRadians),
					x * sin(rotationRadians) + y * cos(rotationRadians), z);
		}
	};
}
