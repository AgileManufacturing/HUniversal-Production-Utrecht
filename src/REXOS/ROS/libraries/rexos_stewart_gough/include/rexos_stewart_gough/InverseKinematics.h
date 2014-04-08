/**
 * @file InverseKinematics.h
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

#pragma once

#include <rexos_datatypes/Point3D.h>
#include <rexos_datatypes/MotorRotation.h>
#include <rexos_datatypes/DeltaRobotMeasures.h>
#include <rexos_stewart_gough/InverseKinematicsModel.h>
#include <rexos_utilities/Utilities.h>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
using namespace std;

// Converts degrees to radians.
//#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
//#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

namespace rexos_stewart_gough{

class InverseKinematics {
	
	
	public:
        struct Point3D {
        	double x, y, z;
        	Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {
        	}
        };

		InverseKinematics(double upperArmLength = 10, double lowerArmLength = 30, double motorAxisToCenterDistance = 10.13):
			upperArmLength(upperArmLength),
			lowerArmLength(lowerArmLength),
			motorAxisToCenterDistance(motorAxisToCenterDistance){
				
		};
		/*
		InverseKinematics(rexos_datatypes::StewartGoughMeasures& measures){
			upperArmLength = measures->hip;
			upperArmLength = measures->ankle;
			motorAxisToCenterDistance = 10.13;
		};*/
		double getAngleForMotor(Point3D moveTo, double motorPositionOnCircle);

		static constexpr double MOTOR_A_POS_ON_CIRCLE = 0;
		static constexpr double MOTOR_B_POS_ON_CIRCLE = 120;
		static constexpr double MOTOR_C_POS_ON_CIRCLE = 240;


	private:
        double upperArmLength;
        double lowerArmLength;
        double motorAxisToCenterDistance;

        vector< vector<double> > getIdentityMatrix();
        vector< vector<double> > getPointMatrix(Point3D point);
        vector< vector<double> > getRotationMatrix(double degrees);
        vector< vector<double> > getTransalationMatrix(double x, double y);
        vector< vector<double> > multiplyMatrix(vector< vector<double> > matrixA, int rowsA, int colsA, vector< vector<double> > matrixB, int rowsB, int colsB);

        double calculateAngle(double d2);
        double calculateCircleIntersectionX(double CenterDistance, double radiusOne, double radiusTwo);
        double calculateCircleDistanceD(Point3D motorOrgin, Point3D effectorJointA);
        double calculateAB(Point3D enginePosition, Point3D jointPosition);

        void deleteMatrixArray(double**, int rows, int cols);

};
}
