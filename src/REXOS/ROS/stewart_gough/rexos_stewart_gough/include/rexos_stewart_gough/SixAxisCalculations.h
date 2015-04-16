#include <iostream>
#include <cmath>
#include <stdexcept>
#include <vector>
#include "rexos_logger/rexos_logger.h"
#include <rexos_stewart_gough/StewartGoughLocation.h>
#include <matrices/Matrices.h>

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
		SixAxisCalculations(double upperArmLength, double lowerArmLength, 
				double motorAxisToCenterDistance, double effectorJointtoCenterDistance, 
				double effectorJointOffset, double motorJointOffset,
				double maxJointAngle):
			upperArmLength(upperArmLength),
			lowerArmLength(lowerArmLength),
			baseRadius(motorAxisToCenterDistance),
			motorJointOffset(motorJointOffset),
			effectorRadius(effectorJointtoCenterDistance),
			effectorJointOffset(effectorJointOffset),
			maxJointAngle(maxJointAngle)
			{
				maxJointAngle = 15.0 * 3.141592 / 180;
				sphereCircleRadius = std::sin(maxJointAngle);
				REXOS_INFO_STREAM("sphereCircleRadius: " << sphereCircleRadius);
	
				//hasValidJointAngles(Vector4(20, -50 - 100, 0, 1), Vector4(-23.81, -51.82, -264.14, 1), Vector4(-23.81 - -37.10, -51.82 - -46.98, -264.14 - -250.00, 1));
				//hasValidJointAngles(Vector4(20, -50 - 100, 0, 1), Vector4(20, -50 - 100, -200, 1), Vector4(1, 0, 0, 1));
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
		
		double sphereCircleRadius;
		
		static constexpr double numberOfGroups = 3;
		
		bool hasValidJointAngles(Vector3 upperArmLowerArmJoint, Vector3 lowerArmEffectorJoint, Vector3 effectorJointAxis);
		double getRemainingZAngle(double yAngle);
		double getAngleForGroup(int jointIndex);
		Matrix4 getEffectorRotationMatrix(StewartGoughLocation preRotatedEffectorLocation, double groupAngle);
		Vector2 getIntersectionPoint(Vector2 pointA, double radiusA, Vector2 pointB, double radiusB);
		Vector3 getEffectorJointPosition(StewartGoughLocation preRotatedEffectorLocation, JointPositionInGroup jointPosition, Matrix4 rotationMatrix);
		Vector3 getMotorAxisPosition(JointPositionInGroup jointPosition);
		double getMotorAngle(StewartGoughLocation effectorLocation, int motorIndex);
};
}
