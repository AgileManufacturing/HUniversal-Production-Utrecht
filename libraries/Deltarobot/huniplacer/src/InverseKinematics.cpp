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

