#include "rexos_stewart_gough/SixAxisCalculations.h"
#include "rexos_utilities/Utilities.h"
#include "rexos_utilities/Utilities.h"
#include <matrices/Matrices.h>

namespace rexos_stewart_gough {
/*double SixAxisCalculations::calculateAB(Point3D enginePosition, Point3D jointPosition){
	//double ac = sqrt(pow(enginePosition.x - jointPosition.x, 2) + pow(enginePosition.y - jointPosition.y, 2));



	double ac = enginePosition.x - jointPosition.x; //TODO was x

	//std::cout << "AC: " << ac << std::endl;

	return sqrt(pow(lowerArmLength, 2) + pow(ac, 2));
}
/*
double SixAxisCalculations::getEffectorJointAngle(Point3D effectorJointPosition, Point3D neighbourEffectorJointPosition, double groupPositionOnCircle, double motorPositionOnCircle, double motorAngle){

	//Step 1: determinate the two vectors and transalte them so their intersection lies on the origin
	Point3D vectorEffector(	neighbourEffectorJointPosition.x - effectorJointPosition.x,
							neighbourEffectorJointPosition.y - effectorJointPosition.y,
							neighbourEffectorJointPosition.z - effectorJointPosition.z);

	// The motors are positioned on the circle in a different angle than the effector joints.
	// Get the right motor angles for the given effector joint angle.
	double rad = DEGREES_TO_RADIANS(-groupPositionOnCircle);

	//std::cout << "Angle: " << (-groupPositionOnCircle) << std::endl;







	//Calculate the needed translation on the x axis
	double xTrans = tan(DEGREES_TO_RADIANS((motorPositionOnCircle -groupPositionOnCircle))) * motorAxisToCenterDistance;
	//double xTrans = tan(motorPositionOnCircle -motorGroupPositionOnCircle) * motorAxisToCenterDistance;

	double yTrans = cos(DEGREES_TO_RADIANS((motorPositionOnCircle -groupPositionOnCircle))) * motorAxisToCenterDistance;
	//double xTrans = tan(motorPositionOnCircle -motorGroupPositionOnCircle) * motorAxisToCenterDistance;


	//std::cout << "xTrans: " << xTrans << std::endl;
	//std::cout << "yTrans: " << yTrans << std::endl;

	double upperArmJointY = cos(motorAngle) * upperArmLength;
	double upperArmJointZ = sin(motorAngle) * upperArmLength;
	Point3D upperArmJointPosition(xTrans, upperArmJointY + yTrans, upperArmJointZ);


	//std::cout << "upperArmJointPosition: " << upperArmJointPosition << std::endl;




	//Identity
	double matrixIdentity[] = MATRIX_IDENTITY_3X3;

	//Rotation
	double matrixRotation[3*3];
	get3x3RotationMatrix(matrixRotation, rad);







	//double matrixTransalation[3*3];
	//get3x3TransalationMatrix(matrixTransalation, xTrans, -motorAxisToCenterDistance);


	//Calculate the complete rotation and translation matrix, and create the point matrix
	double matrixRotationAndIdentity[3*3];
	getMultiplyMatrix(matrixRotationAndIdentity, matrixIdentity, 3, 3, matrixRotation, 3, 3);

	//double calculationMatrix[3*3];
	//getMultiplyMatrix(calculationMatrix, matrixTransalation, 3, 3, matrixRotationAndIdentity, 3, 3);

	double rotatedPointMatrix[3*1];
	get1x3PointMatrix(rotatedPointMatrix, upperArmJointPosition);



	//Calculate the translated and rotated position of the 3d rotated point from step 1
	double resultPoint[3*1];
	getMultiplyMatrix(resultPoint, matrixRotationAndIdentity, 3, 3, rotatedPointMatrix, 3, 1);




	Point3D rotatedUpperArmJointPosition(
	resultPoint[GET_INDEX(0, 0, 1)] - effectorJointPosition.x,
	resultPoint[GET_INDEX(1, 0, 1)] - effectorJointPosition.y,
	upperArmJointPosition.z - effectorJointPosition.z);


		//Point3D rotatedUpperArmJointPositionTemp(
	//resultPoint[GET_INDEX(0, 0, 1)],
	//resultPoint[GET_INDEX(1, 0, 1)],
	//upperArmJointPosition.z);


	//std::cout << "rotated UpperArmJoint pos: " << rotatedUpperArmJointPositionTemp << std::endl;

	//std::cout << "neighbourEffectorJointPosition: " << neighbourEffectorJointPosition << std::endl;

	//std::cout << "effectorJointPosition: " << effectorJointPosition << std::endl;
























	//double motorX = sin(rad) * motorAxisToCenterDistance;
	//double motorY = cos(rad) * motorAxisToCenterDistance;

	//Point3D vectorMotor(motorX - effectorJointPosition.x, motorY - effectorJointPosition.y, 0 - effectorJointPosition.z);

	return getAngleBetween(rotatedUpperArmJointPosition, vectorEffector);
}

bool SixAxisCalculations::isValidPosition(Point3D effectorJointPosition, Point3D neighbourEffectorJointPosition, double motorAngle, double groupPositionOnCircle, double motorPositionOnCircle){
	if(__isnan(motorAngle) == true){
		return false;
	}


	double angle = abs(HALF_PI - getEffectorJointAngle(effectorJointPosition, neighbourEffectorJointPosition, groupPositionOnCircle, motorPositionOnCircle, motorAngle));
	//std::cout << "Critical angle: " << RADIANS_TO_DEGREES(angle) << std::endl;
	if(angle > maxJointAngle){
		return false;
	}
	return true;
}


double SixAxisCalculations::getAngleForMotor(Point3D moveTo, double groupPositionOnCircle, double motorJointPositionOnCircle, double effectorJointPositionOnCircle, double rotationX, double rotationY, double rotationZ){
	// The six-axis math in four steps
	// 
	// Step 1: Calculate the effector joint position for the given engine and based on the moveTo parameter which is the center of the effector.
	// Step 2: 3d rotate the effector joint the given amount of degrees in the x, y and z plane around the center of the effector.
	// Step 3: Translate the effector joint as if the engine would be at position 0, 0, 0.
	// 		Step 3a: Rotate the joint (around the z axis) for the amount of degrees to motor would need to reach 0 degrees.
	// 		Step 3b: Translate the joint the same amount the engine would need to reach x = 0.
	// Step 4: Calculate the motor angle using inverse kinematics.



	// Step 1: Effector joint position calculation

	double xJoint, yJoint;

	//Get the effector joint position based on the effector center and effector joint position on the circle
	xJoint = sin(DEGREES_TO_RADIANS(effectorJointPositionOnCircle)) * effectorJointtoCenterDistance;
	yJoint = cos(DEGREES_TO_RADIANS(effectorJointPositionOnCircle)) * effectorJointtoCenterDistance;



	//xJoint = getXEffectorPositionForEngineRotation(degreesToRadians(effectorJointPositionOnCircle));
	//yJoint = getYEffectorPositionForEngineRotation(degreesToRadians(effectorJointPositionOnCircle));


	double xMotorPos, yMotorPos;

	double radius = motorAxisToCenterDistance / cos(abs(groupPositionOnCircle - motorJointPositionOnCircle));

	radius = motorAxisToCenterDistance;

	xMotorPos = sin(DEGREES_TO_RADIANS(motorJointPositionOnCircle)) * radius;
	yMotorPos = cos(DEGREES_TO_RADIANS(motorJointPositionOnCircle)) * radius;



	//std::cout << "XJoint = (" << xJoint + moveTo.x << ", " << yJoint + moveTo.y << ", " << moveTo.z << ")" << std::endl;

//std::cout << "motorX: " << xMotorPos << " motorY: " << yMotorPos << std::endl;


	// Step 2: 3d rotation around the center of the effector in the x, y and z plane

	//Rotation will be calculated as if the effector center is in the origin of the coordinate system (Relative)
	Point3D moveToOrigin(xJoint, yJoint, 0);


	/*

	vector< vector<double> > matrixFull3DRotation = get4x4RotationMatrix(rotationX, rotationY, rotationZ);

	vector< vector<double> > pointMatrix = get1x4PointMatrix(moveToOrigin);

	vector< vector<double> > resultMatrix = multiplyMatrix(matrixFull3DRotation, 4, 4, pointMatrix, 4, 1);

	*/

	/*double matrixFull3DRotation[4*4];
	get4x4RotationMatrix(matrixFull3DRotation, rotationX, rotationY, rotationZ);
	double pointMatrix[4*1];
	get1x4PointMatrix(pointMatrix, moveToOrigin);


	double resultMatrix[4*1];
	getMultiplyMatrix(resultMatrix, matrixFull3DRotation, 4, 4, pointMatrix, 4, 1);


	//Construct the rotated point and transalate it to the real position in the coordinate system (Absolute)
	Point3D rotatedPoint(resultMatrix[GET_INDEX(0, 0, 1)] + moveTo.x, resultMatrix[GET_INDEX(1, 0, 1)] + moveTo.y, resultMatrix[GET_INDEX(2, 0, 1)] + moveTo.z);

	//std::cout << resultMatrix[GET_INDEX(0, 0, 1)] << std::endl;
	//std::cout << resultMatrix[GET_INDEX(1, 0, 1)] << std::endl;
	//std::cout << resultMatrix[GET_INDEX(2, 0, 1)] << std::endl;

	//std::cout << "Rotated point: "<< rotatedPoint << std::endl;


	//Ugly cache method, so 6 matrix 3d rotations can be skipped in the calculation of the safety angles
	if(effectorJointPositionOnCircle == EFFECTOR_JOINT_A1_POS){
		this->effectorJointPositionCache[0] = rotatedPoint;
	} else if(effectorJointPositionOnCircle == EFFECTOR_JOINT_A2_POS){
		this->effectorJointPositionCache[1] = rotatedPoint;
	} else if(effectorJointPositionOnCircle == EFFECTOR_JOINT_B1_POS){
		this->effectorJointPositionCache[2] = rotatedPoint;
	} else if(effectorJointPositionOnCircle == EFFECTOR_JOINT_B2_POS){
		this->effectorJointPositionCache[3] = rotatedPoint;
	} else if(effectorJointPositionOnCircle == EFFECTOR_JOINT_C1_POS){
		this->effectorJointPositionCache[4] = rotatedPoint;
	} else if(effectorJointPositionOnCircle == EFFECTOR_JOINT_C2_POS){
		this->effectorJointPositionCache[5] = rotatedPoint;
	}






	// Step 3: Rotate around the z axis and translate the joint position as if the engine would be at position 0, 0, 0

	//Prepare matrixes


	/*
	vector< vector<double> >  matrixIdentity = get3x3IdentityMatrix();
	vector< vector<double> >  matrixRotation = get3x3RotationMatrix(degreesToRadians(groupPositionOnCircle));
	 */

	/*//double matrixIdentity[] = MATRIX_IDENTITY_3X3;
	double matrixRotation[3*3];
	get3x3RotationMatrix(matrixRotation, DEGREES_TO_RADIANS(groupPositionOnCircle));


	//Point to rotate from the orgin
		double rotatedPointMatrix[3*1];
		get1x3PointMatrix(rotatedPointMatrix, rotatedPoint);


		//Prepare transformation
		double xTrans = tan(DEGREES_TO_RADIANS((groupPositionOnCircle - motorJointPositionOnCircle))) * motorAxisToCenterDistance;

		double yTrans = -motorAxisToCenterDistance;



		//Do matrix rotation
		double resultPoint[3*1];
		getMultiplyMatrix(resultPoint, matrixRotation, 3, 3, rotatedPointMatrix, 3, 1);


		//Create result point and do transformation
		SixAxisCalculations::Point3D moveToTransalated( resultPoint[GET_INDEX(0, 0, 1)] + xTrans, resultPoint[GET_INDEX(1, 0, 1)] + yTrans, rotatedPoint.z);

		//std::cout << moveToTransalated << std::endl;





		SixAxisCalculations::Point3D engine(0, 0, 0);

	// Step 4: Inverse kinematics.

		//std::cout << "moveToTransalated: " << moveToTransalated << std::endl;


	Point3D circlePlaneVector(1, 0, 0);

	Point3D sphereCenterVector(engine.x - moveToTransalated.x, engine.y - moveToTransalated.y, engine.z  - moveToTransalated.z);

	double sphereRadius = lowerArmLength;

	double dotProduct = (circlePlaneVector.x * (0 - sphereCenterVector.x)) +
						(circlePlaneVector.y * (0 - sphereCenterVector.y)) +
						(circlePlaneVector.z * (0 - sphereCenterVector.z));


	//std::cout << "dot product: " << dotProduct << std::endl;
	if(abs(dotProduct) > sphereRadius){
		//std::cout << "No Intersection!" << std::endl;
	} else {
		//std::cout << "Intersection!" << std::endl;
	}





	double ab = calculateAB(engine,  moveToTransalated);



	double ab_new = sqrt((lowerArmLength*lowerArmLength) - abs(dotProduct*dotProduct));

	//std::cout << "AB new: " << ab_new << std::endl;
	//std::cout << "AB: " << ab << std::endl;


	double dotAbs = abs(dotProduct);

	Point3D dotproductTimesPlaneVector(dotAbs * sphereCenterVector.x, dotAbs * sphereCenterVector.y, dotAbs *sphereCenterVector.z);

	Point3D planeCircleCenter(engine.x, moveToTransalated.y, moveToTransalated.z);




	//c_s + d*n.
	double d_new = calculateCircleDistanceD(engine, planeCircleCenter);

	double d = calculateCircleDistanceD(engine, moveToTransalated);

	//std::cout << "circlePlane center: " << planeCircleCenter << std::endl;

	//std::cout << "d new: " << d_new << std::endl;
	//std::cout << "d: " << d << std::endl;


	double xd = calculateCircleIntersectionX(d_new,  ab_new, upperArmLength);



	Point3D intersection = getIntersectionPoint(0, 0, upperArmLength, planeCircleCenter.y, planeCircleCenter.z, ab_new);

	if(intersection.x == -1){
		//std::cout << "no intersection with circles" << std::endl;
		return NAN;
	} else {
		//std::cout << "Intersection with circles!!" << std::endl;
		//std::cout << "AT: " << intersection << std::endl;


		Point3D vectorBase(0, 0 + 100, 0);



		double angle1  = atan2( vectorBase.z*intersection.y - vectorBase.y*intersection.z, vectorBase.z*intersection.z + vectorBase.y*intersection.y );


		//double angle = getAngleBetween(vectorBase, intersection);

		//std::cout << "Angle: " << RADIANS_TO_DEGREES(angle) << std::endl;
		//std::cout << "Angle1: " << RADIANS_TO_DEGREES(angle1) << std::endl;
		return angle1;
	}




	//std::cout << "x: " << xd << std::endl;


	double d2 = d - xd;

	//return angle; //calculateAngle(xd);
}*/


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
	// the circle comprising the possible position of the upper arm
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
	double rotationAngle = std::atan2(relativeUpperArmLowerArmIntersectionPoint.y, relativeUpperArmLowerArmIntersectionPoint.x);
	double motorRotationAngle = rexos_utilities::degreesToRadians(180) + rotationAngle;
	return motorRotationAngle;
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
}
