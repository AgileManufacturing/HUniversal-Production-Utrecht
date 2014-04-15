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
using namespace std;
  
  namespace rexos_stewart_gough{
  
// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
  
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
  
  
class SixAxisCalculations {
  
    public:
        struct Point3D {
            double x, y, z;
            Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {
            }
        };
  
        SixAxisCalculations(double upperArmLength = 10, double lowerArmLength = 30, double motorAxisToCenterDistance = 10.13, double effectorJointtoCenterDistance = 5.50):
            upperArmLength(upperArmLength),
            lowerArmLength(lowerArmLength),
            motorAxisToCenterDistance(motorAxisToCenterDistance),
            effectorJointtoCenterDistance(effectorJointtoCenterDistance){
        }
  
        double getAngleForMotor(Point3D moveTo, double motorPositionOnCircle);
        double getAngleForMotor(Point3D moveTo, double motorGroupPositionOnCircle, double motorPositionOnCircle, double rotationX, double rotationY, double rotationZ);
  
        double * getAngles(double angles[6], Point3D moveTo, double xRotation, double yRotation, double zRotation);
        double * getAngles(double angles[6], Point3D moveTo);
  
        bool isValidMove(double angles[6]);
  
        //74.396 == magic number
        static constexpr double MOTOR_A_POS_ON_CIRCLE = 0;
        static constexpr double MOTOR_A1_POS_ON_CIRCLE = 0 - 37.198;
        static constexpr double MOTOR_A2_POS_ON_CIRCLE = 0 + 37.198;
  
        static constexpr double MOTOR_B_POS_ON_CIRCLE = 120;
        static constexpr double MOTOR_B1_POS_ON_CIRCLE = 120 - 37.198;
        static constexpr double MOTOR_B2_POS_ON_CIRCLE = 120 + 37.198;
  
        static constexpr double MOTOR_C_POS_ON_CIRCLE = 240;
        static constexpr double MOTOR_C1_POS_ON_CIRCLE = 240 - 37.198;
        static constexpr double MOTOR_C2_POS_ON_CIRCLE = 240 + 37.198;
  
  
    private:
        double upperArmLength;
        double lowerArmLength;
        double motorAxisToCenterDistance;
        double effectorJointtoCenterDistance;
  
        vector< vector<double> > get3DIdentityMatrix();
        vector< vector<double> > get3DRotationMatrix(double xRotation, double yRotation, double zRotation);
        vector< vector<double> > get3DRotationMatrixX(double xRotation);
        vector< vector<double> > get3DRotationMatrixY(double yRotation);
        vector< vector<double> > get3DRotationMatrixZ(double zRotation);
        vector< vector<double> > get3DTransalationMatrix(Point3D point);
        vector< vector<double> > get3DPointMatrix(Point3D point);
  
        vector< vector<double> > getIdentityMatrix();
        vector< vector<double> > getPointMatrix(Point3D point);
        vector< vector<double> > getRotationMatrix(double degrees);
        vector< vector<double> > getTransalationMatrix(double x, double y);
  
        vector< vector<double> > multiplyMatrix(vector< vector<double> > matrixA, int rowsA, int colsA, vector< vector<double> > matrixB, int rowsB, int colsB);
  
        double getXEffectorPositionForEngineRotation(double rotation);
        double getYEffectorPositionForEngineRotation(double rotation);
  
        double calculateAngle(double d2);
        double calculateCircleIntersectionX(double CenterDistance, double radiusOne, double radiusTwo);
        double calculateCircleDistanceD(Point3D motorOrgin, Point3D effectorJointA);
        double calculateAB(Point3D enginePosition, Point3D jointPosition);
};
  }  
#endif /* SIXAXISMATH_H_ */ 
