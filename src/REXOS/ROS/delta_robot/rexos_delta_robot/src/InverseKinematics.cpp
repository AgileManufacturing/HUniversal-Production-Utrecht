/**
 * @file InverseKinematics.cpp
 * @brief Inverse kinematics implementation. Based on work from Viacheslav Slavinsky.\n
 * conventions sitting in front of delta robot:\n
 * x-axis goes from left to right\n
 * y-axis goes from front to back\n
 * z-axis goes from bottom to top\n
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

#include <cassert>
#include <cmath>
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>
#include <rexos_motor/MotorRotation.h>
#include <rexos_delta_robot/InverseKinematics.h>
#include <rexos_delta_robot/InverseKinematicsException.h>
#include <rexos_utilities/Utilities.h>

namespace rexos_delta_robot {
	/**
	 * Constructor for inverse kinematics.
	 * 
	 * @param deltaRobotMeasures The measures of the deltarobot configuration.
	 **/
	InverseKinematics::InverseKinematics(rexos_delta_robot::DeltaRobotMeasures & deltaRobotMeasures) :
			InverseKinematicsModel(deltaRobotMeasures){
	}

	InverseKinematics::~InverseKinematics(void){
	}

	/**
	 * Translates a point to an angle for a motor.
	 * 
	 * @param destinationPoint Point where the midpoint of the effector is wanted.
	 * @param motorLocation Angle of the motor on the z axis where 0 radians is directly in front of the deltarobot.
	 * 
	 * @return The angle, in radians, the motor should move to.
	 **/
	double InverseKinematics::motorAngle(Vector3 destinationPoint, double motorLocation) const {
		// add the height of the effector (and thus move the destination point upwards 
		// to the point where the lower arms connect with the effector
		destinationPoint.z += deltaRobotMeasures.effectorHeight;
		// Rotate the destination point so calculations can be made as if the motor is always in front
		// (rotating the point places it in the same position relative to the front motor
		// as it would be relative to the motor indicated by motor_angle).
		
		Matrix4 rotationMatrix;
		rotationMatrix.rotateZ(-motorLocation);
		
		Vector3 destinationPointRotatedAroundZAxis = rotationMatrix * destinationPoint;
		
		// Places the point towards the "ankle to effector connection".
		destinationPointRotatedAroundZAxis.y -= deltaRobotMeasures.effectorRadius;
		
		// Places the point relative to a motor in (x,y,z) = (0, 0, 0).
		destinationPointRotatedAroundZAxis.y += deltaRobotMeasures.baseRadius;

		double distanceMotorToEffectorOnYAndZAxis = sqrt(pow(destinationPointRotatedAroundZAxis.y, 2)
				+ pow(destinationPointRotatedAroundZAxis.z, 2));

		// Checks if the "ankle to effector connection" is directly to the left of, to the right of, or in the motor.
		if(distanceMotorToEffectorOnYAndZAxis == 0){
			throw InverseKinematicsException("point out of range", destinationPoint);
		}

		// To calculate alpha, the angle between actuator arm and goal vector.
		double alphaAcosInput = (
				pow(destinationPointRotatedAroundZAxis.x, 2)
				- pow(deltaRobotMeasures.ankleLength, 2) + pow(deltaRobotMeasures.hipLength, 2)
				+ pow(distanceMotorToEffectorOnYAndZAxis, 2)
				)/(
				2 
				* deltaRobotMeasures.hipLength 
				* distanceMotorToEffectorOnYAndZAxis);

		if(alphaAcosInput < -1 || alphaAcosInput > 1){
			throw InverseKinematicsException("point out of range", destinationPoint);
		}

		// The required angle between actuator arm and goal vector.
		double alpha = acos(alphaAcosInput);
		
		// The required angle between the base and goal vector.
		double beta = atan2(destinationPointRotatedAroundZAxis.z, destinationPointRotatedAroundZAxis.y);
		
		// The required angle between actuator arm and base (0 degrees).
		double rho = beta - alpha;

		double hipAnkleAngle = asin(abs(destinationPointRotatedAroundZAxis.x) / deltaRobotMeasures.ankleLength);
		if (hipAnkleAngle > deltaRobotMeasures.maxAngleHipAnkle) {
			throw InverseKinematicsException("angle between hip and ankle is out of range", destinationPoint);
		}

		return rho;
	}

	/**
	 * Translates a point to the motor rotations.
	 *
	 * @param destinationPoint The destination point.
	 * @param rotations Array of MotorRotation objects, will be adjusted by the function to the correct rotations per motor.
	 **/
	std::vector<rexos_motor::MotorRotation> InverseKinematics::destinationPointToMotorRotations(const Vector3& destinationPoint) const{
		// Adding 180 degrees switches 0 degrees for the motor from the midpoint of the engines to directly opposite.
		// When determining motorAngle the degrees determine the position of the engines:
		// 	  0 degrees: the hip from this motor moves on the yz plane
		//  120 degrees: this motor is located 120 degrees counter clockwise of the 0 degrees motor when looking at the side the effector is not located
		//  240 degrees: this motor is located 240 degrees counter clockwise of the 0 degrees motor when looking at the side the effector is not located
		std::vector<rexos_motor::MotorRotation> output;
		rexos_motor::MotorRotation rotation;
		
		rotation.angle = rexos_utilities::degreesToRadians(180) + motorAngle(destinationPoint, rexos_utilities::degreesToRadians(1 * 120));
		output.push_back(rotation);
		rotation.angle = rexos_utilities::degreesToRadians(180) + motorAngle(destinationPoint, rexos_utilities::degreesToRadians(0 * 120));
		output.push_back(rotation);
		rotation.angle = rexos_utilities::degreesToRadians(180) + motorAngle(destinationPoint, rexos_utilities::degreesToRadians(2 * 120));
		output.push_back(rotation);
		
		return output;
	}
}
