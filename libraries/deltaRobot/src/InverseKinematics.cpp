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

#include <DeltaRobot/InverseKinematics.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>
#include <DeltaRobot/InverseKinematicsException.h>
#include <DataTypes/MotorRotation.h>

#include <Utilities/Utilities.h>

/**
 * InverseKinematics.cpp -> An implementation of the kinematics model
 * 
 * ideas from Viacheslav Slavinsky are used
 * conventions:
 * sitting in front of delta robot
 * x-axis goes from left to right
 * y-axis goes from front to back
 * z-axis goes from bottom to top
 * point (0,0,0) lies in the middle of all the motors at the motor's height
 **/

namespace DeltaRobot {
	InverseKinematics::InverseKinematics(const double base, const double hip,
			const double effector, const double ankle,
			const double maxAngleHipAnkle) :
			InverseKinematicsModel(base, hip, effector, ankle, maxAngleHipAnkle) {
	}

	InverseKinematics::InverseKinematics(DataTypes::DeltaRobotMeasures & drm) :
			InverseKinematicsModel(drm.base, drm.hip, drm.effector, drm.ankle, drm.maxAngleHipAnkle) {
	}	

	InverseKinematics::~InverseKinematics(void) {
	}

	#define square(x) ((x)*(x))

	/**
	 * Translates a point to an angle for a motor.
	 * @param destinationPoint point where the midpoint of the effector is wanted.
	 * @param motorLocation angle of the motor on the z axis where 0 radians is directly in front of the deltarobot.
	 * @return angle the motor should move to.
	 **/
	double InverseKinematics::motorAngle(const DataTypes::Point3D<double>& destinationPoint,
			double motorLocation) const {

		//rotate the destination point so calculations can be made as if the motor is always in front
		//(rotating the point places it in the same position relative to the front motor
		//as it would be relative to the motor indicated by motor_angle)
		DataTypes::Point3D<double> destinationPointRotatedAroundZAxis =
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
		double alphaAcosInput = (square(destinationPointRotatedAroundZAxis.x)
				- square(ankle) + square(hip)
				+ square(distanceMotorToEffectorOnYAndZAxis))
				/ (2 * hip * distanceMotorToEffectorOnYAndZAxis);

		if (alphaAcosInput < -1 || alphaAcosInput > 1) {
			throw InverseKinematicsException("point out of range",
					destinationPoint);
		}

		//the required angle between actuator arm and goal vector
		double alpha = acos(alphaAcosInput);
		//the required angle between the base and goal vector
		double beta = atan2(destinationPointRotatedAroundZAxis.z,
				destinationPointRotatedAroundZAxis.y);
		//the required angle between actuator arm and base (0 degrees)
		double rho = beta - alpha;

		double hipAnkleAngle = asin(
				abs(destinationPointRotatedAroundZAxis.x) / ankle);
		if (hipAnkleAngle > maxAngleHipAnkle) {
			//std::cerr << "hipAnkleAngle > maxAngleHipAnklel " << Utilities::deg(hipAnkleAngle) << " > " << Utilities::deg(maxAngleHipAnkle) << std::endl;
			throw InverseKinematicsException(
					"angle between hip and ankle is out of range",
					destinationPoint);
		}

		return rho;
	}
	#undef square


	/**
	 * Translates a point to a motion.
	 * @param destinationPoint destination point.
	 * @param motionPointer output parameter, the resulting motion is stored here.
	 * @return true on success, false otherwise.
	 **/
	void InverseKinematics::pointToMotion(const DataTypes::Point3D<double>& destinationPoint, DataTypes::DeltaRobotRotation& rotation) const {
		/**
		 * Adding 180 degrees switches 0 degrees for the motor from the 
		 * midpoint of the engines to directly opposite.
		 * When determining motorAngle the degrees determine the position of the engines:
		 * 	  0 degrees: the hip from this motor moves on the yz plane
		 *  120 degrees: this motor is located 120 degrees counter clockwise of the 0 degrees motor 
		 * when looking at the side the effector is not located
		 *  240 degrees: this motor is located 240 degrees counter clockwise of the 0 degrees motor
		 * when looking at the side the effector is not located
		 **/

		rotation.rotations[0].angle = Utilities::rad(180)
				+ motorAngle(destinationPoint, Utilities::rad(1 * 120));
		rotation.rotations[1].angle = Utilities::rad(180)
				+ motorAngle(destinationPoint, Utilities::rad(0 * 120));
		rotation.rotations[2].angle = Utilities::rad(180)
				+ motorAngle(destinationPoint, Utilities::rad(2 * 120));

		rotation.rotations[0].acceleration =
			rotation.rotations[1].acceleration =
			rotation.rotations[2].acceleration = 
			Utilities::rad(3600);

		rotation.rotations[0].deceleration =
			rotation.rotations[1].deceleration =
			rotation.rotations[2].deceleration = 
			Utilities::rad(3600);
	}
}

