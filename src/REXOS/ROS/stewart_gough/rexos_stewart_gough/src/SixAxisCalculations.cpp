#include "rexos_stewart_gough/SixAxisCalculations.h"
#include "rexos_utilities/Utilities.h"
#include "rexos_utilities/Utilities.h"
#include <matrices/Matrices.h>
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
	REXOS_INFO_STREAM("lowerArmDirection: " << lowerArmDirection);
	
	Vector4 lowerArmNormal(-lowerArmDirection.z, 0, lowerArmDirection.x, 0);
	lowerArmNormal.normalize();
	REXOS_INFO_STREAM("lowerArmNormal: " << lowerArmNormal);
	
	///////////////////////
	// calculate the "Z" rotation 
	///////////////////////
	double upperArmJointZAngle = std::atan2(upperArmJointAxis.y, upperArmJointAxis.x);
	double effectorJointZAngle = std::atan2(effectorJointAxis.y, effectorJointAxis.x);
	// assuming we can evenly destribued the Z rotation over the two joints
	double angleZ = (effectorJointZAngle - upperArmJointZAngle) / 2;
	REXOS_INFO_STREAM("angleZ: " << angleZ * 180 / 3.141592);
	
	double lowerArmYAngle = std::atan2(lowerArmNormal.z, lowerArmNormal.x);
	REXOS_INFO_STREAM("lowerArmYAngle: " << lowerArmYAngle * 180 / 3.141592);
	
	///////////////////////
	// calculate the "Y" rotation for the upperArmLowerArmJoint
	///////////////////////
	double upperArmJointAxisYAngle = std::atan2(upperArmJointAxis.z, upperArmJointAxis.x);
	double upperArmJointYAngle = upperArmJointAxisYAngle - lowerArmYAngle;
	
	REXOS_INFO_STREAM("upperArmJointAxisYAngle: " << upperArmJointAxisYAngle * 180 / 3.141592);
	REXOS_INFO_STREAM("upperArmJointYAngle: " << upperArmJointYAngle * 180 / 3.141592);
	
	if(std::abs(upperArmJointYAngle) > maxJointAngle) {
		return false;
	}
	///////////////////////
	// calculate the "Y" rotation for the lowerArmLowerArmJoint
	///////////////////////
	double effectorJointAxisYAngle = std::atan2(effectorJointAxis.z, effectorJointAxis.x);
	double effectorJointYAngle = effectorJointAxisYAngle - lowerArmYAngle;
	
	REXOS_INFO_STREAM("effectorJointAxisYAngle: " << effectorJointAxisYAngle * 180 / 3.141592);
	REXOS_INFO_STREAM("effectorJointYAngle: " << effectorJointYAngle * 180 / 3.141592);
	
	if(std::abs(effectorJointYAngle) > maxJointAngle) {
		return false;
	}
	
	
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
	REXOS_INFO_STREAM("yCoordinateInCircel: " << yCoordinateInCircel);
	
	double remainingZ = std::sqrt(1 - std::pow(yCoordinateInCircel, 2));
	double remainingZAngle = std::asin(remainingZ) / (90 * 3.141592 / 180) * maxJointAngle;
	REXOS_INFO_STREAM("remainingZ: " << remainingZ);
	REXOS_INFO_STREAM("remainingZAngle: " << remainingZAngle * 180 / 3.141592);
	return remainingZAngle;
}
	
double SixAxisCalculations::getAngleForGroup(int jointIndex) {
	// intentional int devision
	int jointGroup = jointIndex / 2;
	return rexos_utilities::degreesToRadians(360 / numberOfGroups * jointGroup);
}

Vector2 SixAxisCalculations::getIntersectionPoint(Vector2 pointA, double radiusA, Vector2 pointB, double radiusB) {
	ROS_INFO_STREAM("---pointA: " << pointA);
	ROS_INFO_STREAM("---radiusA: " << radiusA);
	ROS_INFO_STREAM("---pointB: " << pointB);
	ROS_INFO_STREAM("---radiusB: " << radiusB);
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
	ROS_INFO_STREAM("--deltaAB: " << deltaAB);
	ROS_INFO_STREAM("--distance: " << distance);
	ROS_INFO_STREAM("--intersectionLineDistance: " << intersectionLineDistance);
	
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
	ROS_INFO_STREAM("--intersectionPointA: " << intersectionPointA);
	ROS_INFO_STREAM("--intersectionPointB: " << intersectionPointB);
	
	return intersectionPointB;
}

Vector3 SixAxisCalculations::getEffectorJointPosition(StewartGoughLocation preRotatedEffectorLocation, 
		JointPositionInGroup jointPosition, double groupAngle) {
	Vector3 offsetVector(0, -effectorRadius, 0);
	if(jointPosition == JointPositionInGroup::left) {
		offsetVector.x -= effectorJointOffset;
	} else {
		// right joint
		offsetVector.x += effectorJointOffset;
	}
	
	// rotate the offsetVector to match the effector location
	Matrix4 rotationMatrix;
	// rotate to the proper effector group
	rotationMatrix.rotateZ(groupAngle);
	rotationMatrix.rotateX(preRotatedEffectorLocation.rotationX);
	rotationMatrix.rotateY(preRotatedEffectorLocation.rotationY);
	rotationMatrix.rotateZ(preRotatedEffectorLocation.rotationZ);
	REXOS_ERROR_STREAM("realEffectorJointPosition: " << 
			preRotatedEffectorLocation.location + rotationMatrix * offsetVector);
	// rotate back to the original position of the effector group
	rotationMatrix.rotateZ(-groupAngle);
	
	Vector3 rotatedOffsetVector = rotationMatrix * offsetVector;
	REXOS_ERROR_STREAM("rotatedOffsetVector: " << rotatedOffsetVector);
	return preRotatedEffectorLocation.location + rotatedOffsetVector;
}

Vector3 SixAxisCalculations::getMotorAxisPosition(JointPositionInGroup jointPosition) {
	Vector3 offsetVector(0, -baseRadius, 0);
	if(jointPosition == JointPositionInGroup::left) {
		offsetVector.x -= motorJointOffset;
	} else {
		// right joint
		offsetVector.x += motorJointOffset;
	}
	return offsetVector;
}

double SixAxisCalculations::getMotorAngle(StewartGoughLocation effectorLocation, int motorIndex) {
	ROS_WARN_STREAM("motorIndex: " << motorIndex);
	// Get angle for this effector joint group
	double groupAngle = getAngleForGroup(motorIndex);
	ROS_INFO_STREAM("groupAngle: " << groupAngle);
	
	Matrix4 rotationMatrix;
	// Rotate to the correct joint group
	rotationMatrix.rotateZ(-groupAngle);
	
	StewartGoughLocation preRotatedEffectorLocation(rotationMatrix * effectorLocation.location, 
			effectorLocation.rotationX, effectorLocation.rotationY, effectorLocation.rotationZ);
	ROS_INFO_STREAM("preRotatedEffectorLocation: " << preRotatedEffectorLocation);
	
	JointPositionInGroup jointPosition;
	if((motorIndex % 2) == 0) {
		jointPosition = JointPositionInGroup::left;
	ROS_INFO_STREAM("jointPosition: " << jointPosition);
	} else {
		jointPosition = JointPositionInGroup::right;
	ROS_INFO_STREAM("jointPosition: " << jointPosition);
	}
	
	Vector3 motorAxisPosition = getMotorAxisPosition(jointPosition);
	ROS_INFO_STREAM("motorAxisPosition: " << motorAxisPosition);
	Vector3 effectorJointPosition = getEffectorJointPosition(preRotatedEffectorLocation, jointPosition, groupAngle);
	ROS_INFO_STREAM("effectorJointPosition: " << effectorJointPosition);
	
	// Project the sphere comprising the possible positions of the lower arm on 
	// the circle comprising the possible positions of the upper arm
	// Determine the x distance between the motorAxisPosition and the effectorJointPosition
	double deltaX = motorAxisPosition.x - effectorJointPosition.x;
	ROS_INFO_STREAM("deltaX: " << deltaX);
	double innerCircleRadius = std::sqrt(std::pow(lowerArmLength, 2) - std::pow(deltaX, 2));
	ROS_INFO_STREAM("innerCircleRadius: " << innerCircleRadius);
	
	Vector2 upperArmLowerArmIntersectionPoint = getIntersectionPoint(
			Vector2(motorAxisPosition.y, motorAxisPosition.z), upperArmLength, 
			Vector2(effectorJointPosition.y, effectorJointPosition.z), innerCircleRadius);
	ROS_INFO_STREAM("upperArmLowerArmIntersectionPoint: " << upperArmLowerArmIntersectionPoint);
	
	Vector2 relativeUpperArmLowerArmIntersectionPoint = Vector2(
			upperArmLowerArmIntersectionPoint.x - motorAxisPosition.y, 
			upperArmLowerArmIntersectionPoint.y - motorAxisPosition.z);
	ROS_INFO_STREAM("relativeUpperArmLowerArmIntersectionPoint: " << relativeUpperArmLowerArmIntersectionPoint);
	
	// Determine the angle for the motor
	// The x and y value are multiplied by -1 to invert the coordinate system for the axis, this because the origin of the unit circle is on the other side (rotated by 180 degrees).
	double motorRotationAngle = std::atan2(-relativeUpperArmLowerArmIntersectionPoint.y, -relativeUpperArmLowerArmIntersectionPoint.x);
	
	Vector3 upperArmLowerArmJoint(motorAxisPosition.x, upperArmLowerArmIntersectionPoint.x, upperArmLowerArmIntersectionPoint.y);
	REXOS_WARN_STREAM("upperArmLowerArmJoint" << upperArmLowerArmJoint);
	Matrix4 rotationMatrix2;
	rotationMatrix2.rotateX(preRotatedEffectorLocation.rotationX);
	rotationMatrix2.rotateY(preRotatedEffectorLocation.rotationY);
	rotationMatrix2.rotateZ(preRotatedEffectorLocation.rotationZ);
	Vector3 effectorAxis = rotationMatrix * Vector3(1, 0, 0);
	REXOS_WARN_STREAM("effectorAxis" << effectorAxis);
	if(hasValidJointAngles(upperArmLowerArmJoint, effectorJointPosition, effectorAxis) == false) {
		REXOS_ERROR("INvalid angle!!");
		return std::nan("");
	} else {
		return motorRotationAngle;
	}
}

SixAxisCalculations::EffectorMove SixAxisCalculations::getMotorAngles(StewartGoughLocation moveTo) {
	ROS_WARN_STREAM("moveTo: " << moveTo);
	EffectorMove result;

	result.moveTo = moveTo;
	result.validMove = true;

	for(int i = 0; i < 6; i++) {
		result.angles[i] = getMotorAngle(moveTo, i);
		ROS_INFO_STREAM("Angle " << (double) i << ": " << result.angles[i]);
		if(std::isnan(result.angles[i])) {
			result.validMove = false;
			break;
		}
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

bool SixAxisCalculations::checkJoints(){
	double screwheadRadius = 3.5;
	double rodEndRadius = 3.5;
	
	Vector3 originCone(0, 0, 0); 
	Vector3 midpointScrewhead(0, 0, 3);	
	Vector3 midpointScrewEnd(-3, 0, 0);
	
	int amountOfPoints = 20;
	
	Vector3 points[amountOfPoints];
	ParameticEquation equations[amountOfPoints];
	
	for(int i = 0; i < 360; i += (360/amountOfPoints)){
		points[i/(360/amountOfPoints)].x = 3;
		points[i/(360/amountOfPoints)].y = cos(i) * screwheadRadius;
		points[i/(360/amountOfPoints)].z = sin(i) * screwheadRadius;
		
		equations[(360/amountOfPoints)].vector1 = points[i/(360/amountOfPoints)];
		equations[(360/amountOfPoints)].vector2 = points[i/(360/amountOfPoints)] - midpointScrewEnd;
		
	
	}
	
	//check for the position of the screw (let the screw rotate instead of the rod, this makes it easier for the math)
	
	//check wether the line is to far away from the midpoint of the rod
	
	return true;
}
}
