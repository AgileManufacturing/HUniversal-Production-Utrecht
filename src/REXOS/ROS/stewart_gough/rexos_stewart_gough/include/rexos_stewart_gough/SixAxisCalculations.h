#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "rexos_logger/rexos_logger.h"
#include <rexos_stewart_gough/StewartGoughLocation.h>

#pragma once

namespace rexos_stewart_gough{
class SixAxisCalculations {

	public:
		enum JointPositionInGroup {
			left,
			right
		};
	
	
		struct EffectorMove {
			StewartGoughLocation moveTo;
			bool validMove;
			double angles[6];
		};

		SixAxisCalculations(double upperArmLength = 100.00, double lowerArmLength = 300.00, 
				double motorAxisToCenterDistance = 101.30, double effectorJointtoCenterDistance = 43.14, 
				double maxJointAngle = 0.46):
			upperArmLength(upperArmLength),
			lowerArmLength(lowerArmLength),
			baseRadius(motorAxisToCenterDistance),
			effectorRadius(effectorJointtoCenterDistance),
			maxJointAngle(maxJointAngle),
			effectorJointOffset(35.0),
			motorJointOffset(35.0)
			{
		}

		EffectorMove getMotorAngles(StewartGoughLocation moveTo);
		bool checkPath(StewartGoughLocation from, StewartGoughLocation to);


		/*static constexpr double EFFECTOR_MAGIC_NUMBER = 37.198;

		/*
		 * Can be calculated using:
		 * double distanceBetweenMotors = (sin(degreesToRadians(EFFECTOR_MAGIC_NUMBER)) * effectorJointtoCenterDistance) * 2;
		 * MOTOR_MAGIC_NUMBER = radiansToDegrees(asin((elbowGroupDistance/2) / motorAxisToCenterDistance));
		 */
		static constexpr double MOTOR_MAGIC_NUMBER = 19.1624;


		static constexpr double GROUP_A_POS 			= 0;
		static constexpr double EFFECTOR_JOINT_A1_POS 	= 0 - 37.198;
		static constexpr double EFFECTOR_JOINT_A2_POS 	= 0 + 37.198;
		static constexpr double MOTOR_A1_POS 			= 0 - 19.1624;
		static constexpr double MOTOR_A2_POS 			= 0 + 19.1624;

		static constexpr double GROUP_B_POS 			= 120;
		static constexpr double EFFECTOR_JOINT_B1_POS 	= 120 - 37.198;
		static constexpr double EFFECTOR_JOINT_B2_POS 	= 120 + 37.198;
		static constexpr double MOTOR_B1_POS 			= 120 - 19.1624;
		static constexpr double MOTOR_B2_POS 			= 120 + 19.1624;

		static constexpr double GROUP_C_POS 			= 240;
		static constexpr double EFFECTOR_JOINT_C1_POS 	= 240 - 37.198;
		static constexpr double EFFECTOR_JOINT_C2_POS 	= 240 + 37.198;
		static constexpr double MOTOR_C1_POS 			= 240 - 19.1624;
		static constexpr double MOTOR_C2_POS 			= 240 + 19.1624;

	private:
		double upperArmLength;
		double lowerArmLength;
		double baseRadius;
		double motorJointOffset;
		double effectorRadius;
		double effectorJointOffset;
		double maxJointAngle;
		
		static constexpr double numberOfGroups = 3;
		
		double getAngleForGroup(int jointIndex);
		Vector2 getIntersectionPoint(Vector2 pointA, double radiusA, Vector2 pointB, double radiusB);
		Vector3 getEffectorJointPosition(StewartGoughLocation preRotatedEffectorLocation, JointPositionInGroup jointPosition, double groupAngle);
		Vector3 getMotorAxisPosition(JointPositionInGroup jointPosition);
		double getMotorAngle(StewartGoughLocation effectorLocation, int motorIndex);
		
		

        /*//Vector operations
        double getAngleBetween(Vector3 vectorOne, Vector3 vectorTwo);

        //Motor angle calculation and validation
        double getAngleForMotor(Point3D moveTo, double groupPositionOnCircle, double motorJointPositionOnCircle, double effectorJointPositionOnCircle, double rotationX, double rotationY, double rotationZ);
        bool isValidPosition(Point3D relativeJointPosition, Point3D relativeNeighbourJointPosition, double motorAngle, double motorGroupPositionOnCircle, double motorPositionOnCircle);
        double getEffectorJointAngle(Point3D effectorJointPosition, Point3D neighbourEffectorJointPosition, double groupPositionOnCircle, double motorPositionOnCircle, double motorAngle);

        //Inverse kinematics
        double calculateAngle(double d2);
        double calculateCircleIntersectionX(double CenterDistance, double radiusOne, double radiusTwo);
        double calculateCircleDistanceD(Point3D motorOrgin, Point3D effectorJointA);
        double calculateAB(Point3D enginePosition, Point3D jointPosition);*/
};
}
