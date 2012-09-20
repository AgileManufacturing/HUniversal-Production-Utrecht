//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        inverseKinematics.cpp
// File:           inverse kinematics implementation. Based on work from Viacheslav Slavinsky
// Original:       Lukas Vermond & Kasper van Nieuwland
// Edited by:      Daan Veltman
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of inverseKinematics.cpp.
//
// inverseKinematics.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// inverseKinematics.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with inverseKinematics.cpp.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <huniplacer/InverseKinematics.h>

#include <cmath>
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>
#include <huniplacer/InverseKinematicsException.h>

#include <huniplacer/utils.h>

namespace huniplacer {
InverseKinematics::InverseKinematics(const double base, const double hip,
		const double effector, const double ankle,
		const double MaxAngleHipAnkle) :
		InverseKinematicsModel(base, hip, effector, ankle, MaxAngleHipAnkle) {
}

InverseKinematics::~InverseKinematics(void) {
}

#define square(x) ((x)*(x))
double InverseKinematics::motorAngle(const Point3D& destinationPoint,
		double motorLocation) const {
	//ideas from Viacheslav Slavinsky are used
	//conventions:
	//	sitting in front of delta robot
	//	x-axis goes from left to right
	//	y-axis goes from front to back
	//	z-axis goes from bottom to top
	//	point (0,0,0) lies in the middle of all the motors at the motor's height

	//rotate the destination point so calculations can be made as if the motor is always in front
	//(rotating the point places it in the same position relative to the front motor
	//as it would be relative to the motor indicated by motor_angle)
	Point3D destinationPointRotatedAroundZAxis =
			destinationPoint.rotateAroundZAxis(-motorLocation);

	//places the point towards the "ankle to effector connection"
	destinationPointRotatedAroundZAxis.y -= effector;
	//places the point relative to a motor in (x,y,z) = (0,0,0)
	destinationPointRotatedAroundZAxis.y += base;

	//why is this?
	double distanceMotorToEffectorOnYAndZAxis = sqrt(
			square(destinationPointRotatedAroundZAxis.y)
					+ square(destinationPointRotatedAroundZAxis.z));

	//checks if the "ankle to effector connection" is directly
	//to the left of, to the right of, or in the motor
	if (distanceMotorToEffectorOnYAndZAxis == 0) {
		throw InverseKinematicsException("point out of range",
				destinationPoint);
	}

	//to calculate alpha, the angle between actuator arm and goal vector
	double alpha_acos_input = (square(destinationPointRotatedAroundZAxis.x)
			- square(ankle) + square(hip)
			+ square(distanceMotorToEffectorOnYAndZAxis))
			/ (2 * hip * distanceMotorToEffectorOnYAndZAxis);

	if (alpha_acos_input < -1 || alpha_acos_input > 1) {
		throw InverseKinematicsException("point out of range",
				destinationPoint);
	}

	//the required angle between actuator arm and goal vector
	double alpha = acos(alpha_acos_input);
	//the required angle between the base and goal vector
	double beta = atan2(destinationPointRotatedAroundZAxis.z,
			destinationPointRotatedAroundZAxis.y);
	//the required angle between actuator arm and base (0 degrees)
	double rho = beta - alpha;

	double hip_ankle_angle = asin(
			abs(destinationPointRotatedAroundZAxis.x) / ankle);
	if (hip_ankle_angle > maxAngleHipAnkle) {
		throw InverseKinematicsException(
				"angle between hip and ankle is out of range",
				destinationPoint);
	}

	return rho;
}
#undef square

void InverseKinematics::pointToMotion(const Point3D& destinationPoint,
		motionf& motionPointer) const {
	//adding 180 degrees switches 0 degrees from the engine midpoint to
	//directly opposite
	motionPointer.angles[0] = utils::rad(180)
			+ motorAngle(destinationPoint, utils::rad(0 * 120));
	motionPointer.angles[1] = utils::rad(180)
			+ motorAngle(destinationPoint, utils::rad(1 * 120));
	motionPointer.angles[2] = utils::rad(180)
			+ motorAngle(destinationPoint, utils::rad(2 * 120));

	motionPointer.acceleration[0] = motionPointer.acceleration[1] =
			motionPointer.acceleration[2] = utils::rad(3600);
	motionPointer.deceleration[0] = motionPointer.deceleration[1] =
			motionPointer.deceleration[2] = utils::rad(3600);
}
}

