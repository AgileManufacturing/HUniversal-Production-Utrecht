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

namespace huniplacer {
/// @brief 3 dimensional point class
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
	 * @brief calculates the euclidean distance between *this and p
	 * @return distance
	 **/
	inline double distance(const Point3D& p) const {
		double dx = x - p.x;
		double dy = y - p.y;
		double dz = z - p.z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}

	/**
	 * @brief rotate over the Y axis
	 * @param phi rotation in radians
	 **/
	inline Point3D rotateAroundYAxis(double rotationRadians) const {
		return Point3D(x * cos(rotationRadians) - z * sin(rotationRadians), y,
				x * sin(rotationRadians) + z * cos(rotationRadians));
	}

	/**
	 * @brief rotate over the Z axis
	 * @param phi rotation in radians
	 **/
	inline Point3D rotateAroundZAxis(double rotationRadians) const {
		return Point3D(x * cos(rotationRadians) - y * sin(rotationRadians),
				x * sin(rotationRadians) + y * cos(rotationRadians), z);
	}
};
}
