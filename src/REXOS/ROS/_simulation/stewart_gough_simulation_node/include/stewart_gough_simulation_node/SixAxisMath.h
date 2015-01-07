/*
 * SixAxisMath.h
 *
 *  Created on: 4 apr. 2014
 *      Author: Rolf
 */

#ifndef SIXAXISMATH_H_
#define SIXAXISMATH_H_

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
using namespace std;

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)


class SixAxisMath {

	public:
        struct Point3D {
        	double x, y, z;
        	Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {
        	}
        };

		SixAxisMath(double upperArmLength = 10, double lowerArmLength = 30, double motorAxisToCenterDistance = 10.13):
			upperArmLength(upperArmLength),
			lowerArmLength(lowerArmLength),
			motorAxisToCenterDistance(motorAxisToCenterDistance){

		}

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

#endif /* SIXAXISMATH_H_ */
