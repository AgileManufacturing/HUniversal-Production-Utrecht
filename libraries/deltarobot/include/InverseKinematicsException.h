//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        	huniplacer
// File:           	inverseKinematicsException.h
// Description:    	exception thrown if an invalid value is calculated during inverse kinematics
// Author:         	Lukas Vermond & Kasper van Nieuwland
// Edited by:		Daan Veltman
// Notes:          	-
//
// License:        	GNU GPL v3
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

namespace huniplacer {
/**
 * @brief instances of deltarobot will throw this exception whenever they fail to convert a point
 **/
class InverseKinematicsException: public std::runtime_error {
private:
	Point3D notConvertablePoint;

public:
	InverseKinematicsException(const char* exceptionMessage,
			Point3D destinationPoint) :
			std::runtime_error(exceptionMessage), notConvertablePoint(
					destinationPoint) {
	}

	virtual ~InverseKinematicsException(void) throw () {
	}

	/**
	 * @brief used to access the point that could not be converted
	 **/
	Point3D getNotConvertablePoint(void) {
		return notConvertablePoint;
	}
};
}
