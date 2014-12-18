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
		
		struct ParameticEquation {
			Vector3 vector1;
			Vector3 vector2;
		};
		
		struct ArmJointAngles {
			double angleZ;
			double effectorArmAngleY;
			double upperArmAngleY;
		};

		SixAxisCalculations(double upperArmLength, double lowerArmLength, 
				double motorAxisToCenterDistance, double effectorJointtoCenterDistance, 
				double effectorJointOffset, double motorJointOffset,
				double maxJointAngle):
			upperArmLength(upperArmLength),
			lowerArmLength(lowerArmLength),
			baseRadius(motorAxisToCenterDistance),
			effectorRadius(effectorJointtoCenterDistance),
			maxJointAngle(maxJointAngle),
			effectorJointOffset(effectorJointOffset),
			motorJointOffset(motorJointOffset)
			{
				getArmJointAngles(Vector4(20, -50 - 100, 0, 1), Vector4(-23.81, -51.82, -264.14, 1), Vector4(-23.81 - -37.10, -51.82 - -46.98, -264.14 - -250.00, 1));
		}

		EffectorMove getMotorAngles(StewartGoughLocation moveTo);
		bool checkPath(StewartGoughLocation from, StewartGoughLocation to);
		
	private:
		double upperArmLength;
		double lowerArmLength;
		double baseRadius;
		double motorJointOffset;
		double effectorRadius;
		double effectorJointOffset;
		double maxJointAngle;
		
		static constexpr double numberOfGroups = 3;
		
		ArmJointAngles getArmJointAngles(Vector4 upperArmLowerArmJoint, Vector4 lowerArmEffectorJoint, Vector4 effectorJointAxis);
		double getAngleForGroup(int jointIndex);
		Vector2 getIntersectionPoint(Vector2 pointA, double radiusA, Vector2 pointB, double radiusB);
		Vector3 getEffectorJointPosition(StewartGoughLocation preRotatedEffectorLocation, JointPositionInGroup jointPosition, double groupAngle);
		Vector3 getMotorAxisPosition(JointPositionInGroup jointPosition);
		double getMotorAngle(StewartGoughLocation effectorLocation, int motorIndex);
		bool checkJoints();
};
}
