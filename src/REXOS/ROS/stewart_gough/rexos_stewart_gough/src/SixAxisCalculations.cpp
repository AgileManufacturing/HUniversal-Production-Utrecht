#include "rexos_stewart_gough/SixAxisCalculations.h"
#include "rexos_utilities/Utilities.h"
#include "rexos_utilities/Utilities.h"
#define _USE_MATH_DEFINES
#include <math.h>


namespace rexos_stewart_gough {
bool SixAxisCalculations::hasValidJointAngles(Vector3 upperArmLowerArmJoint, Vector3 lowerArmEffectorJoint, Vector3 effectorJointAxis) {
	// we know that the upper arm will remain straight
	Vector3 upperArmJointAxis(1, 0, 0);
	
	///////////////////////
	// calulate the normal vector of the lower arm, pointing to the right-ish
	///////////////////////
	Vector3 lowerArmDirection = lowerArmEffectorJoint - upperArmLowerArmJoint;
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "lowerArmDirection: " << lowerArmDirection);
	
	Vector4 lowerArmNormal(-lowerArmDirection.z, 0, lowerArmDirection.x, 0);
	lowerArmNormal.normalize();
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "lowerArmNormal: " << lowerArmNormal);
	
	double lowerArmYAngle = std::atan2(lowerArmNormal.z, lowerArmNormal.x);
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "lowerArmYAngle: " << rexos_utilities::radiansToDegrees(lowerArmYAngle));
	
	///////////////////////
	// calculate the "Y" rotation for the upperArmLowerArmJoint
	///////////////////////
	double upperArmJointAxisYAngle = std::atan2(upperArmJointAxis.z, upperArmJointAxis.x);
	double upperArmJointYAngle = upperArmJointAxisYAngle - lowerArmYAngle;
	
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "upperArmJointAxisYAngle: " << rexos_utilities::radiansToDegrees(upperArmJointAxisYAngle));
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "upperArmJointYAngle: " << rexos_utilities::radiansToDegrees(upperArmJointYAngle));
	
	if(std::abs(upperArmJointYAngle) > stewartGoughMeasures.maxJointAngle) {
		return false;
	}
	
	///////////////////////
	// calculate the "Y" rotation for the lowerArmLowerArmJoint
	///////////////////////
	double effectorJointAxisYAngle = std::atan2(effectorJointAxis.z, effectorJointAxis.x);
	double effectorJointYAngle = effectorJointAxisYAngle - lowerArmYAngle;
	
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "effectorJointAxisYAngle: " << rexos_utilities::radiansToDegrees(effectorJointAxisYAngle));
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "effectorJointYAngle: " << rexos_utilities::radiansToDegrees(effectorJointYAngle));
	
	if(std::abs(effectorJointYAngle) > stewartGoughMeasures.maxJointAngle) {
		return false;
	}
	
	
	///////////////////////
	// calculate the "Z" rotation 
	///////////////////////
	double upperArmJointZAngle = std::atan2(upperArmJointAxis.y, upperArmJointAxis.x);
	double effectorJointZAngle = std::atan2(effectorJointAxis.y, effectorJointAxis.x);
	// assuming we can evenly destribued the Z rotation over the two joints
	double angleZ = (effectorJointZAngle - upperArmJointZAngle) / 2;
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "angleZ: " << rexos_utilities::radiansToDegrees(angleZ));
	
	double remainingUpperArmJointZAngle = getRemainingZAngle(upperArmJointYAngle);
	double remainingEffectorJointZAngle = getRemainingZAngle(effectorJointYAngle);
	
	if(remainingUpperArmJointZAngle + remainingEffectorJointZAngle >= angleZ) {
		return true;
	} else {
		return false;
	}
}
double SixAxisCalculations::getRemainingZAngle(double yAngle) {
	double yCoordinateInCircel = std::sin(yAngle) / sphereCircleRadius;
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "yCoordinateInCircel: " << yCoordinateInCircel);
	
	double remainingZ = std::sqrt(1 - std::pow(yCoordinateInCircel, 2));
	double remainingZAngle = std::asin(remainingZ) / rexos_utilities::degreesToRadians(90) * stewartGoughMeasures.maxJointAngle;
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "remainingZ: " << remainingZ);
	REXOS_DEBUG_STREAM_NAMED("sixAxisJointCheck", "remainingZAngle: " << rexos_utilities::radiansToDegrees(remainingZAngle));
	return remainingZAngle;
}
	
double SixAxisCalculations::getAngleForGroup(int jointIndex) {
	// intentional int devision
	int jointGroup = jointIndex / 2;
	return rexos_utilities::degreesToRadians(360 / numberOfGroups * jointGroup);
}

Matrix4 SixAxisCalculations::getEffectorRotationMatrix(StewartGoughLocation preRotatedEffectorLocation, double groupAngle){
	// rotate the offsetVector to match the effector location
	Matrix4 rotationMatrix;
	// rotate to the proper effector group
	rotationMatrix.rotateZ(groupAngle);
	rotationMatrix.rotateX(preRotatedEffectorLocation.rotationX);
	rotationMatrix.rotateY(preRotatedEffectorLocation.rotationY);
	rotationMatrix.rotateZ(preRotatedEffectorLocation.rotationZ);
	// rotate back to the original position of the effector group
	rotationMatrix.rotateZ(-groupAngle);
	return rotationMatrix;
}

Vector2 SixAxisCalculations::getIntersectionPoint(Vector2 pointA, double radiusA, Vector2 pointB, double radiusB) {
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "pointA: " << pointA);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "radiusA: " << radiusA);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "pointB: " << pointB);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "radiusB: " << radiusB);
	Vector2 deltaAB = pointB - pointA;
	//Determine the straight-Line distance between the centers.
	double distance = pointA.distance(pointB);
	
	if(distance > (radiusA + radiusB)){
		// No intersection possible because the points are to far apart
		return Vector2(0, 0);
	} else if(distance < abs(radiusA - radiusB)){
		// No intersection possible because circle B is completely inside circle A or vice versa
		return Vector2(0, 0);
	}
	
	// Determine the distance from point A to the intersection line
	double intersectionLineDistance = (std::pow(distance, 2) - std::pow(radiusB, 2) + std::pow(radiusA, 2)) / (2 * distance);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "deltaAB: " << deltaAB);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "distance: " << distance);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "intersectionLineDistance: " << intersectionLineDistance);
	
	// Determine the coordinates of the intersection point of the intersection line and the line AB
	Vector2 interSectionLineOnLineAB = pointA + (deltaAB * intersectionLineDistance / distance);
	
	//Determine the distance from crossPoint To either of the
	//intersection points.
	double intersectionLineHalfLength = sqrt(std::pow(radiusA, 2) - std::pow(intersectionLineDistance, 2));
	
	// Determine the intersection line by expressing the intersection line as a vector perpundicular to the line AB
	Vector2 intersectionLine = (deltaAB * (intersectionLineHalfLength / distance));
	intersectionLine = Vector2(-intersectionLine.y, intersectionLine.x);
	
	// Determine the absolute intersection points.
	Vector2 intersectionPointA = interSectionLineOnLineAB + (intersectionLine);
	Vector2 intersectionPointB = interSectionLineOnLineAB + (-intersectionLine);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "intersectionPointA: " << intersectionPointA);
	REXOS_DEBUG_STREAM_NAMED("sixAxisIntersectionPoint", "intersectionPointB: " << intersectionPointB);
	
	return intersectionPointB;
}

Vector3 SixAxisCalculations::getEffectorJointPosition(StewartGoughLocation preRotatedEffectorLocation, 
		JointPositionInGroup jointPosition, Matrix4 rotationMatrix) {
	Vector3 offsetVector(0, -stewartGoughMeasures.effectorRadius, stewartGoughMeasures.effectorHeight);
	if(jointPosition == JointPositionInGroup::left) {
		offsetVector.x -= stewartGoughMeasures.effectorJointOffset;
	} else {
		offsetVector.x += stewartGoughMeasures.effectorJointOffset;
	}
	
	Vector3 rotatedOffsetVector = rotationMatrix * offsetVector;
	REXOS_DEBUG_STREAM_NAMED("sixAxisEffectorLocation", "rotatedOffsetVector: " << rotatedOffsetVector);
	return preRotatedEffectorLocation.location + rotatedOffsetVector;
}

Vector3 SixAxisCalculations::getMotorAxisPosition(JointPositionInGroup jointPosition) {
	Vector3 offsetVector(0, -stewartGoughMeasures.baseRadius, 0);
	if(jointPosition == JointPositionInGroup::left) {
		offsetVector.x -= stewartGoughMeasures.motorJointOffset;
	} else {
		// right joint
		offsetVector.x += stewartGoughMeasures.motorJointOffset;
	}
	return offsetVector;
}

double SixAxisCalculations::getMotorAngle(StewartGoughLocation effectorLocation, int motorIndex) {
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "motorIndex: " << motorIndex);
	// Get angle for this effector joint group
	double groupAngle = getAngleForGroup(motorIndex);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "groupAngle: " << groupAngle);
	
	Matrix4 rotationMatrix;
	// Rotate to the correct joint group
	rotationMatrix.rotateZ(-groupAngle);
	
	StewartGoughLocation rotatedEffectorLocation(rotationMatrix * effectorLocation.location, 
			effectorLocation.rotationX, effectorLocation.rotationY, effectorLocation.rotationZ);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "rotatedEffectorLocation: " << rotatedEffectorLocation);
	
	JointPositionInGroup jointPosition;
	if((motorIndex % 2) == 0) {
		jointPosition = JointPositionInGroup::left;
	} else {
		jointPosition = JointPositionInGroup::right;
	}
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "jointPosition: " << jointPosition);
	
	Vector3 motorAxisPosition = getMotorAxisPosition(jointPosition);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "motorAxisPosition: " << motorAxisPosition);
	Matrix4 effectorRotationMatrix = getEffectorRotationMatrix(rotatedEffectorLocation, groupAngle);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "effectorRotationMatrix: " << effectorRotationMatrix);
	Vector3 effectorJointPosition = getEffectorJointPosition(rotatedEffectorLocation, jointPosition, effectorRotationMatrix);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "effectorJointPosition: " << effectorJointPosition);
	
	// Project the sphere comprising the possible positions of the lower arm on 
	// the circle comprising the possible positions of the upper arm
	// Determine the x distance between the motorAxisPosition and the effectorJointPosition
	double deltaX = motorAxisPosition.x - effectorJointPosition.x;
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "deltaX: " << deltaX);
	double innerCircleRadius = std::sqrt(std::pow(stewartGoughMeasures.ankleLength, 2) - std::pow(deltaX, 2));
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "innerCircleRadius: " << innerCircleRadius);
	
	Vector2 upperArmLowerArmIntersectionPoint = getIntersectionPoint(
			Vector2(motorAxisPosition.y, motorAxisPosition.z), stewartGoughMeasures.hipLength, 
			Vector2(effectorJointPosition.y, effectorJointPosition.z), innerCircleRadius);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "upperArmLowerArmIntersectionPoint: " << upperArmLowerArmIntersectionPoint);
	
	Vector2 relativeUpperArmLowerArmIntersectionPoint = Vector2(
			upperArmLowerArmIntersectionPoint.x - motorAxisPosition.y, 
			upperArmLowerArmIntersectionPoint.y - motorAxisPosition.z);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "relativeUpperArmLowerArmIntersectionPoint: " << relativeUpperArmLowerArmIntersectionPoint);
	
	// Determine the angle for the motor
	// The x and y value are multiplied by -1 to invert the coordinate system for the axis, this because the origin of the unit circle is on the other side (rotated by 180 degrees).
	double motorRotationAngle = std::atan2(-relativeUpperArmLowerArmIntersectionPoint.y, -relativeUpperArmLowerArmIntersectionPoint.x);
	
	Vector3 upperArmLowerArmJoint(motorAxisPosition.x, upperArmLowerArmIntersectionPoint.x, upperArmLowerArmIntersectionPoint.y);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "upperArmLowerArmJoint" << upperArmLowerArmJoint);
	Vector3 effectorAxis = effectorRotationMatrix * Vector3(1, 0, 0);
	REXOS_DEBUG_STREAM_NAMED("sixAxisMotorAngle", "effectorAxis" << effectorAxis);
	if(hasValidJointAngles(upperArmLowerArmJoint, effectorJointPosition, effectorAxis) == false) {
		throw std::runtime_error("invalid joint angle");
	} else {
		return motorRotationAngle;
	}
}

SixAxisCalculations::EffectorMove SixAxisCalculations::getMotorAngles(StewartGoughLocation moveTo) {
	ROS_INFO_STREAM("moveTo: " << moveTo);
	EffectorMove result;

	result.moveTo = moveTo;
	
	try {
		for(int i = 0; i < 6; i++) {
			result.angles[i] = getMotorAngle(moveTo, i);
			ROS_INFO_STREAM("Angle " << (double) i << ": " << result.angles[i]);
			if(std::isnan(result.angles[i])) {
				result.validMove = false;
				break;
			}
		}
		result.validMove = true;
	} catch(std::runtime_error ex) {
		result.validMove = false;
	}
	
	return result;
}

bool SixAxisCalculations::checkPath(StewartGoughLocation from, StewartGoughLocation to){
		//Calculate lengths to travel
    	double x_length = to.location.x - from.location.x;
    	double y_length = to.location.y - from.location.y;
    	double z_length = to.location.z - from.location.z;


		double rotationX_length = to.rotationX - from.rotationX;
    	double rotationY_length = to.rotationY - from.rotationY;
    	double rotationZ_length = to.rotationZ - from.rotationZ;


    	double largest_length = (double)	(fabs(x_length) > fabs(y_length) ?
											(fabs(x_length) > fabs(z_length) ? fabs(x_length) : fabs(z_length)) :
											(fabs(y_length) > fabs(z_length) ? fabs(y_length) : fabs(z_length))	);

    	double stepLengthX = x_length / largest_length;
    	double stepLengthY = y_length / largest_length;
    	double stepLengthZ = z_length / largest_length;

		double stepLengthRotationX = rotationX_length / largest_length;
    	double stepLengthRotationY = rotationY_length / largest_length;
    	double stepLengthRotationZ = rotationZ_length / largest_length;


		//std::cout << "path length: " << largest_length << std::endl;

		for(double step = 1; step <= largest_length; step++){

			double x = (from.location.x + stepLengthX * step);
			double y = (from.location.y + stepLengthY * step);
			double z = (from.location.z + stepLengthZ * step);

			double rotationX = (from.rotationX + stepLengthRotationX * step);
			double rotationY = (from.rotationY + stepLengthRotationY * step);
			double rotationZ = (from.rotationZ + stepLengthRotationZ * step);


			EffectorMove move = getMotorAngles(StewartGoughLocation(Vector3(x, y, z), rotationX, rotationY, rotationZ));
			if(!move.validMove){
				REXOS_ERROR_STREAM("invalid position in path: " << move.moveTo << std::endl);
				return false;
			}
		}
        return true;
}
}
