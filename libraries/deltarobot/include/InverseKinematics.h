//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        	huniplacer
// File:           	inverseKinematics.h
// Description:    	inverse kinematics implementation. Based on work from Viacheslav Slavinsky
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

#include <huniplacer/point3.h>
#include <huniplacer/motion.h>
#include <huniplacer/inverseKinematicsModel.h>

namespace huniplacer {
/**
 * @brief an implementation of the kinematics model
 **/
class InverseKinematics: public InverseKinematicsModel {
private:
	/**
	 * @brief translates a point to an angle for a motor
	 * @param destinationPoint point where the midpoint of the effector is wanted
	 * @param motorLocation angle of the motor on the z axis where 0 radians is directly in front of the deltarobot
	 * @return angle the motor should move to
	 **/
	double motorAngle(const Point3D& destinationPoint,
			double motorLocation) const;

public:
	InverseKinematics(const double base, const double hip,
			const double effector, const double ankle,
			const double hip_ankle_angle_max);

	virtual ~InverseKinematics(void);

	/**
	 * @brief translates a point to a motion
	 * @param destinationPoint destination point
	 * @param motionPointer output parameter, the resulting motion is stored here
	 * @return true on success, false otherwise
	 **/
	void pointToMotion(const Point3D& destinationPoint,
			motionf& motionPointer) const;
};
}
