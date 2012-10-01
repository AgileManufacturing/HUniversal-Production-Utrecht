//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           Point3D.h
// Description:    3 dimensional point class
// Author:         Lukas Vermond & Kasper van Nieuwland
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

namespace huniplacer {
	/**
	 * Point3D.h -> 3 dimensional point class
	 **/
	class Point3D {
	public:
		double x, y, z;

		Point3D(double x, double y, double z) :
				x(x), y(y), z(z) {
		}
		~Point3D() {
		}

		Point3D offset(Point3D& p);

		inline Point3D& operator+=(const Point3D& rhs) {
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
			return *this;
		}

		inline const Point3D operator+(const Point3D& rhs) const {
			Point3D res = *this;
			res += rhs;
			return res;
		}

		/**
		 * Calculates the euclidean distance between *this and p
		 * @return distance
		 **/
		inline double distance(const Point3D& p) const {
			double dx = x - p.x;
			double dy = y - p.y;
			double dz = z - p.z;
			return sqrt(dx * dx + dy * dy + dz * dz);
		}

		/**
		 * Rotate the point around the Y axis
		 * @param phi rotation in radians
		 **/
		inline Point3D rotateAroundYAxis(double rotationRadians) const {
			return Point3D(x * cos(rotationRadians) - z * sin(rotationRadians), y,
					x * sin(rotationRadians) + z * cos(rotationRadians));
		}

		/**
		 * Rotate the point around the Z axis
		 * @param phi rotation in radians
		 **/
		inline Point3D rotateAroundZAxis(double rotationRadians) const {
			return Point3D(x * cos(rotationRadians) - y * sin(rotationRadians),
					x * sin(rotationRadians) + y * cos(rotationRadians), z);
		}
	};
}
