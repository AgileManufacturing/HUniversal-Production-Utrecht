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
#include <rexos_datatypes/MotorRotation.h>
#include <rexos_stewart_gough/InverseKinematics.h>
#include <rexos_stewart_gough/InverseKinematicsException.h>



namespace rexos_stewart_gough {

double InverseKinematics::getAngleForMotor(Point3D moveTo, double motorPositionOnCircle){

	double x, y;

	if(motorPositionOnCircle != 0){

		vector< vector<double> > matrixIdentity = getIdentityMatrix();
		vector< vector<double> >  matrixRotation = getRotationMatrix(motorPositionOnCircle);
		vector< vector<double> >  matrixTransalation = getTransalationMatrix(-motorAxisToCenterDistance, 0);
		vector< vector<double> >  matrixRotationAndIdentity = multiplyMatrix(matrixIdentity, 3, 3, matrixRotation, 3, 3);
		vector< vector<double> >  calculationMatrix = multiplyMatrix(matrixTransalation, 3, 3, matrixRotationAndIdentity, 3, 3);
		vector< vector<double> >  pointMatrix = getPointMatrix(moveTo);
		vector< vector<double> >  resultPoint = multiplyMatrix(calculationMatrix, 3, 3, pointMatrix, 3, 1);

		x = resultPoint[0][0];
		y = resultPoint[1][0];

	} else {
		x = moveTo.x - motorAxisToCenterDistance;
		y = moveTo.y;
	}


	InverseKinematics::Point3D engine(0, 0, 0);
	InverseKinematics::Point3D moveToTransalated(x, y, moveTo.z);

	double ab = calculateAB(engine,  moveToTransalated);

	double xd = calculateCircleIntersectionX(std::abs((double) (engine.z - moveToTransalated.z)), upperArmLength, ab);



	double d = calculateCircleDistanceD(engine, moveToTransalated);
	double d2 = d - xd;


	double calculatedAngle = calculateAngle(d2);

	return calculatedAngle;
}




vector< vector<double> > InverseKinematics::getIdentityMatrix(){
	vector< vector<double> > matrix(3, vector<double>(3));

	matrix[0][0] = 1;
	matrix[0][1] = 0;
	matrix[0][2] = 0;

	matrix[1][0] = 0;
	matrix[1][1] = 1;
	matrix[1][2] = 0;

	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = 1;

	return matrix;
}

vector< vector<double> > InverseKinematics::getPointMatrix(InverseKinematics::Point3D point){
	vector< vector<double> > matrix(3, vector<double>(1));
	matrix[0][0] = point.x;
	matrix[1][0] = point.y;
	matrix[2][0] = 1;
	return matrix;
}

vector< vector<double> > InverseKinematics::getRotationMatrix(double degrees){
	double radians = rexos_utilities::degreesToRadians(degrees);

	vector< vector<double> > matrix(3, vector<double>(3));

	matrix[0][0] = cos(radians);
	matrix[0][1] = -sin(radians);
	matrix[0][2] = 0;

	matrix[1][0] = sin(radians);
	matrix[1][1] = cos(radians);
	matrix[1][2] = 0;

	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = 1;

	return matrix;
}

vector< vector<double> > InverseKinematics::getTransalationMatrix(double x, double y){
	vector< vector<double> > matrix(3, vector<double>(3));

	matrix[0][0] = 1;
	matrix[0][1] = 0;
	matrix[0][2] = x;

	matrix[1][0] = 0;
	matrix[1][1] = 1;
	matrix[1][2] = y;

	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = 1;

	return matrix;
}

void InverseKinematics::deleteMatrixArray(double** matrix, int rows, int cols){
	int i;
	for(i = 0; i < rows; i++){
		delete[] matrix[i];
	}
	delete[] matrix;
}



vector< vector<double> > InverseKinematics::multiplyMatrix(vector< vector<double> > matrixA, const int rowsA, const int colsA, vector< vector<double> > matrixB, const int rowsB, const int colsB){


	if (colsA != rowsB) {
		throw std::length_error("MatrixA:Columns did not match MatrixB:Rows!");
	}

	vector< vector<double> > matrixC(rowsA, vector<double>(colsB));

	for (int i = 0; i < rowsA; i++) { // aRow
		for (int j = 0; j < colsB; j++) { // bColumn
			for (int k = 0; k < colsA; k++) { // aColumn
				matrixC[i][j] += matrixA[i][k] * matrixB[k][j];
			}
		}
	}

	return matrixC;
}

double InverseKinematics::calculateAngle(double d2){
	return rexos_utilities::radiansToDegrees((asin(d2/upperArmLength)));
}

double InverseKinematics::calculateCircleIntersectionX(double CenterDistance, double radiusOne, double radiusTwo){
	return ((pow(CenterDistance, 2) - pow(radiusOne, 2) + pow(radiusTwo, 2)) / (2 * CenterDistance));
}

//Euclidean distance
double InverseKinematics::calculateCircleDistanceD(InverseKinematics::Point3D motorOrgin, InverseKinematics::Point3D effectorJointA){
	return sqrt(pow(motorOrgin.x - effectorJointA.x, 2) + pow(motorOrgin.y - effectorJointA.y, 2) +pow(motorOrgin.z - effectorJointA.z, 2));
}

double InverseKinematics::calculateAB(Point3D enginePosition, Point3D jointPosition){
	double ac = enginePosition.y - jointPosition.y; //TODO was x
	return sqrt(pow(lowerArmLength, 2) + pow(ac, 2));
}
	
}
