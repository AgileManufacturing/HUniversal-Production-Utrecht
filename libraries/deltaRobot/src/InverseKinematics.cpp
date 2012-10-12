/**
 * @file InverseKinematics.cpp
 * @brief Inverse kinematics implementation. Based on work from Viacheslav Slavinsky. 
 * conventions sitting in front of delta robot:
 * x-axis goes from left to right
 * y-axis goes from front to back
 * z-axis goes from bottom to top
 * point (0,0,0) lies in the middle of all the motors at the motor's height
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Daan Veltman
 * @author 1.1 Koen Braham
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
 **/

#include <DeltaRobot/InverseKinematics.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>
#include <DeltaRobot/InverseKinematicsException.h>
#include <DataTypes/MotorRotation.h>

#include <Utilities/Utilities.h>

/**
 * InverseKinematics.cpp -> An implementation of the kinematics model.
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
			InverseKinematicsModel(base, hip, effector, ankle, maxAngleHipAnkle){
	}

	InverseKinematics::InverseKinematics(DataTypes::DeltaRobotMeasures & deltaRobotMeasures) :
			InverseKinematicsModel(deltaRobotMeasures.base, deltaRobotMeasures.hip, deltaRobotMeasures.effector, deltaRobotMeasures.ankle, deltaRobotMeasures.maxAngleHipAnkle){
	}	

	InverseKinematics::~InverseKinematics(void){
	}

	#define square(x) ((x)*(x))

	/**
	 * Translates a point to an angle for a motor.
	 * 
	 * @param destinationPoint Point where the midpoint of the effector is wanted.
	 * @param motorLocation Angle of the motor on the z axis where 0 radians is directly in front of the deltarobot.
	 * 
	 * @return angle the motor should move to.
	 **/
	double InverseKinematics::motorAngle(const DataTypes::Point3D<double>& destinationPoint, double motorLocation) const{
		// Rotate the destination point so calculations can be made as if the motor is always in front
		// (rotating the point places it in the same position relative to the front motor
		// as it would be relative to the motor indicated by motor_angle).
		DataTypes::Point3D<double> destinationPointRotatedAroundZAxis =
				destinationPoint.rotateAroundZAxis(-motorLocation);

		// Places the point towards the "ankle to effector connection".
		destinationPointRotatedAroundZAxis.y -= effector;
		
		// Places the point relative to a motor in (x,y,z) = (0, 0, 0).
		destinationPointRotatedAroundZAxis.y += base;

		// TODO: (look at the comment below)
		// why is this?
		double distanceMotorToEffectorOnYAndZAxis = sqrt(square(destinationPointRotatedAroundZAxis.y)
				+ square(destinationPointRotatedAroundZAxis.z));

		// Checks if the "ankle to effector connection" is directly to the left of, to the right of, or in the motor.
		if(distanceMotorToEffectorOnYAndZAxis == 0){
			throw InverseKinematicsException("point out of range", destinationPoint);
		}

		// To calculate alpha, the angle between actuator arm and goal vector.
		double alphaAcosInput = (square(destinationPointRotatedAroundZAxis.x)
				- square(ankle) + square(hip)
				+ square(distanceMotorToEffectorOnYAndZAxis))
				/ (2 * hip * distanceMotorToEffectorOnYAndZAxis);

		if(alphaAcosInput < -1 || alphaAcosInput > 1){
			throw InverseKinematicsException("point out of range", destinationPoint);
		}

		// The required angle between actuator arm and goal vector.
		double alpha = acos(alphaAcosInput);
		
		// The required angle between the base and goal vector.
		double beta = atan2(destinationPointRotatedAroundZAxis.z, destinationPointRotatedAroundZAxis.y);
		
		// The required angle between actuator arm and base (0 degrees).
		double rho = beta - alpha;

		double hipAnkleAngle = asin(abs(destinationPointRotatedAroundZAxis.x) / ankle);
		if (hipAnkleAngle > maxAngleHipAnkle) {
			throw InverseKinematicsException("angle between hip and ankle is out of range", destinationPoint);
		}

		return rho;
	}
	#undef square

	/**
	 * Translates a point to a motion.
	 * 
	   TODO: motionPointer is gone and param rotations is missing.
	 * @param destinationPoint destination point.
	 * @param motionPointer output parameter, the resulting motion is stored here.
	 * 
	 * @return true on success, false otherwise.
	 **/
	void InverseKinematics::pointToMotion(const DataTypes::Point3D<double>& destinationPoint, DataTypes::MotorRotation<double>* (&rotations)[3]) const{
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
		rotations[0]->angle = Utilities::degreesToRadians(180) + motorAngle(destinationPoint, Utilities::degreesToRadians(1 * 120));
		rotations[1]->angle = Utilities::degreesToRadians(180) + motorAngle(destinationPoint, Utilities::degreesToRadians(0 * 120));
		rotations[2]->angle = Utilities::degreesToRadians(180) + motorAngle(destinationPoint, Utilities::degreesToRadians(2 * 120));

		rotations[0]->acceleration = rotations[1]->acceleration = rotations[2]->acceleration = Utilities::degreesToRadians(3600);
		rotations[0]->deceleration = rotations[1]->deceleration = rotations[2]->deceleration = Utilities::degreesToRadians(3600);
	}
}
