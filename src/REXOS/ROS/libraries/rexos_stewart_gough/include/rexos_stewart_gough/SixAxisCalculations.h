/*
 * SixAxisMath.h
 *  Copyright: Rolf Smit
 *  Created on: 4 apr. 2014
 *      Author: Rolf
 */

#ifndef SIXAXISMATH_H_
#define SIXAXISMATH_H_

#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>


// Converts degrees to radians.
#define DEGREES_TO_RADIANS(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define RADIANS_TO_DEGREES(angleRadians) (angleRadians * 180.0 / M_PI)

#define HALF_PI (M_PI/2)
#define GET_INDEX(row, col, maxCols) (row * maxCols + col)



#define MATRIX_IDENTITY_4X4 {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
#define MATRIX_POINT_4X1 	{0, 0, 0, 1}
#define MATRIX_EMPTY_4X1 	{0, 0, 0, 0}
#define MATRIX_EMPTY_4X4 	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#define MATRIX_IDENTITY_3X3 {1, 0, 0, 0, 1, 0, 0, 0, 1}
#define MATRIX_POINT_3X1 	{0, 0, 1}
#define MATRIX_EMPTY_3X1 	{0, 0, 0}
#define MATRIX_EMPTY_3X3 	{0, 0, 0, 0, 0, 0, 0, 0, 0}


class SixAxisCalculations {

	public:
        struct Point3D {
        	double x, y, z;
        	Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {
        	}

        	friend std::ostream &operator<<(std::ostream &stream, Point3D const &p) {
        		return stream << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        	}
        };

		struct EffectorMove {
			Point3D moveTo;
			double effectorRotationX;
			double effectorRotationY;
			double effectorRotationZ;
			bool validMove;
			double angles[6];
		};

		SixAxisCalculations(double upperArmLength = 100.00, double lowerArmLength = 300.00, double motorAxisToCenterDistance = 101.30, double effectorJointtoCenterDistance = 55.00, double maxJointAngle = 0.17):
			upperArmLength(upperArmLength),
			lowerArmLength(lowerArmLength),
			motorAxisToCenterDistance(motorAxisToCenterDistance),
			effectorJointtoCenterDistance(effectorJointtoCenterDistance),
			maxJointAngle(maxJointAngle){
		}

		EffectorMove getMotorAngles(Point3D moveTo, double xRotation, double yRotation, double zRotation);
		double * getAngles(double angles[6], Point3D moveTo, double xRotation, double yRotation, double zRotation);

		bool checkPath(Point3D from, double startRotationX, double startRotationY, double startRotationZ, Point3D to, double endRotationX, double endRotationY, double endRotationZ);


		//
		bool isValidMove(double angles[6]);

		void matrixTest();


		constexpr static double EFFECTOR_MAGIC_NUMBER = 37.198;

		/*
		 * Can be calculated using:
		 * double distanceBetweenMotors = (sin(degreesToRadians(EFFECTOR_MAGIC_NUMBER)) * effectorJointtoCenterDistance) * 2;
		 * MOTOR_MAGIC_NUMBER = radiansToDegrees(asin((elbowGroupDistance/2) / motorAxisToCenterDistance));
		 */
		constexpr static double MOTOR_MAGIC_NUMBER = 19.1624;


		constexpr static double GROUP_A_POS 			= 0;
		constexpr static double EFFECTOR_JOINT_A1_POS 	= 0 - 37.198;
		constexpr static double EFFECTOR_JOINT_A2_POS 	= 0 + 37.198;
		constexpr static double MOTOR_A1_POS 			= 0 - 19.1624;
		constexpr static double MOTOR_A2_POS 			= 0 + 19.1624;

		constexpr static double GROUP_B_POS 			= 120;
		constexpr static double EFFECTOR_JOINT_B1_POS 	= 120 - 37.198;
		constexpr static double EFFECTOR_JOINT_B2_POS 	= 120 + 37.198;
		constexpr static double MOTOR_B1_POS 			= 120 - 19.1624;
		constexpr static double MOTOR_B2_POS 			= 120 + 19.1624;

		constexpr static double GROUP_C_POS 			= 240;
		constexpr static double EFFECTOR_JOINT_C1_POS 	= 240 - 37.198;
		constexpr static double EFFECTOR_JOINT_C2_POS 	= 240 + 37.198;
		constexpr static double MOTOR_C1_POS 			= 240 - 19.1624;
		constexpr static double MOTOR_C2_POS 			= 240 + 19.1624;

	private:
        double upperArmLength;
        double lowerArmLength;
        double motorAxisToCenterDistance;
        double effectorJointtoCenterDistance;
        double maxJointAngle;

        Point3D effectorJointPositionCache[6];

		//Matrix operations
		void getMultiplyMatrix(double result[], double matrixA[], int rowsA, int colsA, double matrixB[], int rowsB, int colsB);


		/*
        //4*4 Matrix functions
        std::vector< std::vector<double> > get4x4IdentityMatrix();
        std::vector< std::vector<double> > get4x4RotationMatrix(double xRotation, double yRotation, double zRotation);
        std::vector< std::vector<double> > get4x4RotationMatrixX(double xRotation);
        std::vector< std::vector<double> > get4x4RotationMatrixY(double yRotation);
        std::vector< std::vector<double> > get4x4RotationMatrixZ(double zRotation);
        std::vector< std::vector<double> > get4x4TransalationMatrix(Point3D point);
        std::vector< std::vector<double> > get1x4PointMatrix(Point3D point);

        //3*3 Matrix functions
        std::vector< std::vector<double> > get3x3IdentityMatrix();
        std::vector< std::vector<double> > get3x3RotationMatrix(double degrees);
        std::vector< std::vector<double> > get3x3TransalationMatrix(double x, double y);
        std::vector< std::vector<double> > get1x3PointMatrix(Point3D point);
		*/

		//4*4 Matrix functions
		void get4x4RotationMatrix(double result[], double xRotation, double yRotation, double zRotation);
		void get4x4RotationMatrixX(double result[], double xRotation);
		void get4x4RotationMatrixY(double result[], double yRotation);
		void get4x4RotationMatrixZ(double result[], double zRotation);
		void get4x4TransalationMatrix(double result[], Point3D point);
		void get1x4PointMatrix(double result[], Point3D point);

		void get3x3RotationMatrix(double result[], double degrees);
		void get3x3TransalationMatrix(double result[], double x, double y);
		void get1x3PointMatrix(double result[], Point3D point);




        //Matrix operations
        std::vector< std::vector<double> > multiplyMatrix(std::vector< std::vector<double> > matrixA, int rowsA, int colsA, std::vector< std::vector<double> > matrixB, int rowsB, int colsB);

        //Vector operations
        double getAngleBetween(Point3D vectorOne, Point3D vectorTwo);

        //Motor angle calculation and validation
        double getAngleForMotor(Point3D moveTo, double groupPositionOnCircle, double effectorJointPositionOnCircle, double rotationX, double rotationY, double rotationZ);
        bool isValidPosition(Point3D relativeJointPosition, Point3D relativeNeighbourJointPosition, double motorAngle, double motorGroupPositionOnCircle, double motorPositionOnCircle);
        double getEffectorJointAngle(Point3D effectorJointPosition, Point3D neighbourEffectorJointPosition, double groupPositionOnCircle, double motorPositionOnCircle, double motorAngle);

        //Inverse kinematics
        double calculateAngle(double d2);
        double calculateCircleIntersectionX(double CenterDistance, double radiusOne, double radiusTwo);
        double calculateCircleDistanceD(Point3D motorOrgin, Point3D effectorJointA);
        double calculateAB(Point3D enginePosition, Point3D jointPosition);
};

#endif /* SIXAXISMATH_H_ */
