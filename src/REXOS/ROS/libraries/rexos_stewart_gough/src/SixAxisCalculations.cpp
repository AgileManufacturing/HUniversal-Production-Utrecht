/*
 * SixAxisMath.cpp
 *  Copyright: Rolf Smit
 *  Created on: 4 apr. 2014
 *      Author: Rolf
 */

#include "rexos_stewart_gough/SixAxisCalculations.h"
using namespace std;

/*
vector< vector<double> > SixAxisCalculations::get4x4IdentityMatrix(){
	vector< vector<double> > matrix(4, vector<double>(4));

		matrix[0][0] = 1;
		matrix[0][1] = 0;
		matrix[0][2] = 0;
		matrix[0][3] = 0;

		matrix[1][0] = 0;
		matrix[1][1] = 1;
		matrix[1][2] = 0;
		matrix[1][3] = 0;

		matrix[2][0] = 0;
		matrix[2][1] = 0;
		matrix[2][2] = 1;
		matrix[2][3] = 0;

		matrix[3][0] = 0;
		matrix[3][1] = 0;
		matrix[3][2] = 0;
		matrix[3][3] = 1;

		return matrix;
}

vector< vector<double> > SixAxisCalculations::get4x4TransalationMatrix(Point3D point){
	vector< vector<double> > matrix(4, vector<double>(4));

		matrix[0][0] = 1;
		matrix[0][1] = 0;
		matrix[0][2] = 0;
		matrix[0][3] = point.x;

		matrix[1][0] = 0;
		matrix[1][1] = 1;
		matrix[1][2] = 0;
		matrix[1][3] = point.y;

		matrix[2][0] = 0;
		matrix[2][1] = 0;
		matrix[2][2] = 1;
		matrix[2][3] = point.z;

		matrix[3][0] = 0;
		matrix[3][1] = 0;
		matrix[3][2] = 0;
		matrix[3][3] = 1;

		return matrix;
}

vector< vector<double> > SixAxisCalculations::get4x4RotationMatrixX(double xRotation){
	double xRadians = xRotation;

	vector< vector<double> > matrix(4, vector<double>(4));

	matrix[0][0] = 1;
	matrix[0][1] = 0;
	matrix[0][2] = 0;
	matrix[0][3] = 0;

	matrix[1][0] = 0;
	matrix[1][1] = cos(xRadians);
	matrix[1][2] = sin(xRadians);
	matrix[1][3] = 0;

	matrix[2][0] = 0;
	matrix[2][1] = -sin(xRadians);
	matrix[2][2] = cos(xRadians);
	matrix[2][3] = 0;

	matrix[3][0] = 0;
	matrix[3][1] = 0;
	matrix[3][2] = 0;
	matrix[3][3] = 1;

	return matrix;
}

vector< vector<double> > SixAxisCalculations::get4x4RotationMatrixY(double yRotation){
	double yRadians = yRotation;

	vector< vector<double> > matrix(4, vector<double>(4));

	matrix[0][0] = cos(yRadians);
	matrix[0][1] = 0;
	matrix[0][2] = -sin(yRadians);
	matrix[0][3] = 0;

	matrix[1][0] = 0;
	matrix[1][1] = 1;
	matrix[1][2] = 0;
	matrix[1][3] = 0;

	matrix[2][0] = sin(yRadians);
	matrix[2][1] = 0;
	matrix[2][2] = cos(yRadians);
	matrix[2][3] = 0;

	matrix[3][0] = 0;
	matrix[3][1] = 0;
	matrix[3][2] = 0;
	matrix[3][3] = 1;

	return matrix;
}

vector< vector<double> > SixAxisCalculations::get4x4RotationMatrixZ(double zRotation){
	double zRadians = zRotation;

	vector< vector<double> > matrix(4, vector<double>(4));

	matrix[0][0] = cos(zRadians);
	matrix[0][1] = sin(zRadians);
	matrix[0][2] = 0;
	matrix[0][3] = 0;

	matrix[1][0] = -sin(zRadians);
	matrix[1][1] = cos(zRadians);
	matrix[1][2] = 0;
	matrix[1][3] = 0;

	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = 1;
	matrix[2][3] = 0;

	matrix[3][0] = 0;
	matrix[3][1] = 0;
	matrix[3][2] = 0;
	matrix[3][3] = 1;

	return matrix;
}

vector< vector<double> > SixAxisCalculations::get1x4PointMatrix(Point3D point){
	vector< vector<double> > matrix(4, vector<double>(1));
	matrix[0][0] = point.x;
	matrix[1][0] = point.y;
	matrix[2][0] = point.z;
	matrix[3][0] = 1;
	return matrix;
}

vector< vector<double> > SixAxisCalculations::get4x4RotationMatrix(double xRotation, double yRotation, double zRotation){

	vector< vector<double> > matrixIdentity = get4x4IdentityMatrix();

	vector< vector<double> > matrixRotationX = get4x4RotationMatrixX(xRotation);
	vector< vector<double> > matrixRotationY = get4x4RotationMatrixY(yRotation);
	vector< vector<double> > matrixRotationZ = get4x4RotationMatrixZ(zRotation);

	vector< vector<double> >  temp;

	temp = multiplyMatrix(matrixIdentity, 4, 4, matrixRotationX, 4, 4);
	temp = multiplyMatrix(temp, 4, 4, matrixRotationY, 4, 4);
	temp = multiplyMatrix(temp, 4, 4, matrixRotationZ, 4, 4);

	return temp;
}

vector< vector<double> > SixAxisCalculations::get3x3IdentityMatrix(){
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

vector< vector<double> > SixAxisCalculations::get1x3PointMatrix(SixAxisCalculations::Point3D point){
	vector< vector<double> > matrix(3, vector<double>(1));
	matrix[0][0] = point.x;
	matrix[1][0] = point.y;
	matrix[2][0] = 1;
	return matrix;
}

vector< vector<double> > SixAxisCalculations::get3x3RotationMatrix(double degrees){
	double radians = degrees;

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

vector< vector<double> > SixAxisCalculations::get3x3TransalationMatrix(double x, double y){
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
*/

void SixAxisCalculations::get4x4TransalationMatrix(double result[], Point3D point){
	result[GET_INDEX(0, 0, 4)] = 1;
	result[GET_INDEX(0, 1, 4)] = 0;
	result[GET_INDEX(0, 2, 4)] = 0;
	result[GET_INDEX(0, 3, 4)] = point.x;

	result[GET_INDEX(1, 0, 4)] = 0;
	result[GET_INDEX(1, 1, 4)] = 1;
	result[GET_INDEX(1, 2, 4)] = 0;
	result[GET_INDEX(1, 3, 4)] = point.y;

	result[GET_INDEX(2, 0, 4)] = 0;
	result[GET_INDEX(2, 1, 4)] = 0;
	result[GET_INDEX(2, 2, 4)] = 1;
	result[GET_INDEX(2, 3, 4)] = point.z;

	result[GET_INDEX(3, 0, 4)] = 0;
	result[GET_INDEX(3, 1, 4)] = 0;
	result[GET_INDEX(3, 2, 4)] = 0;
	result[GET_INDEX(3, 3, 4)] = 1;
}

void SixAxisCalculations::get4x4RotationMatrixX(double result[], double xRotation){
	result[GET_INDEX(0, 0, 4)] = 1;
	result[GET_INDEX(0, 1, 4)] = 0;
	result[GET_INDEX(0, 2, 4)] = 0;
	result[GET_INDEX(0, 3, 4)] = 0;

	result[GET_INDEX(1, 0, 4)] = 0;
	result[GET_INDEX(1, 1, 4)] = cos(xRotation);
	result[GET_INDEX(1, 2, 4)] = sin(xRotation);
	result[GET_INDEX(1, 3, 4)] = 0;

	result[GET_INDEX(2, 0, 4)] = 0;
	result[GET_INDEX(2, 1, 4)] = -sin(xRotation);
	result[GET_INDEX(2, 2, 4)] = cos(xRotation);
	result[GET_INDEX(2, 3, 4)] = 0;

	result[GET_INDEX(3, 0, 4)] = 0;
	result[GET_INDEX(3, 1, 4)] = 0;
	result[GET_INDEX(3, 2, 4)] = 0;
	result[GET_INDEX(3, 3, 4)] = 1;
}

void SixAxisCalculations::get4x4RotationMatrixY(double result[], double yRotation){
	result[GET_INDEX(0, 0, 4)] = cos(yRotation);
	result[GET_INDEX(0, 1, 4)] = 0;
	result[GET_INDEX(0, 2, 4)] = -sin(yRotation);
	result[GET_INDEX(0, 3, 4)] = 0;

	result[GET_INDEX(1, 0, 4)] = 0;
	result[GET_INDEX(1, 1, 4)] = 1;
	result[GET_INDEX(1, 2, 4)] = 0;
	result[GET_INDEX(1, 3, 4)] = 0;

	result[GET_INDEX(2, 0, 4)] = sin(yRotation);
	result[GET_INDEX(2, 1, 4)] = 0;
	result[GET_INDEX(2, 2, 4)] = cos(yRotation);
	result[GET_INDEX(2, 3, 4)] = 0;

	result[GET_INDEX(3, 0, 4)] = 0;
	result[GET_INDEX(3, 1, 4)] = 0;
	result[GET_INDEX(3, 2, 4)] = 0;
	result[GET_INDEX(3, 3, 4)] = 1;
}

void SixAxisCalculations::get4x4RotationMatrixZ(double result[], double zRotation){
	result[GET_INDEX(0, 0, 4)] = cos(zRotation);
	result[GET_INDEX(0, 1, 4)] = sin(zRotation);
	result[GET_INDEX(0, 2, 4)] = 0;
	result[GET_INDEX(0, 3, 4)] = 0;

	result[GET_INDEX(1, 0, 4)] = -sin(zRotation);
	result[GET_INDEX(1, 1, 4)] = cos(zRotation);
	result[GET_INDEX(1, 2, 4)] = 0;
	result[GET_INDEX(1, 3, 4)] = 0;

	result[GET_INDEX(2, 0, 4)] = 0;
	result[GET_INDEX(2, 1, 4)] = 0;
	result[GET_INDEX(2, 2, 4)] = 1;
	result[GET_INDEX(2, 3, 4)] = 0;

	result[GET_INDEX(3, 0, 4)] = 0;
	result[GET_INDEX(3, 1, 4)] = 0;
	result[GET_INDEX(3, 2, 4)] = 0;
	result[GET_INDEX(3, 3, 4)] = 1;
}

void SixAxisCalculations::get1x4PointMatrix(double result[], Point3D point){
	result[GET_INDEX(0, 0, 1)] = point.x;
	result[GET_INDEX(1, 0, 1)] = point.y;
	result[GET_INDEX(2, 0, 1)] = point.z;
	result[GET_INDEX(3, 0, 1)] = 1;
}

void SixAxisCalculations::get4x4RotationMatrix(double result[], double xRotation, double yRotation, double zRotation){

	double matrixIdentity[] = MATRIX_IDENTITY_4X4;


	double matrixRotationX[4*4];
	get4x4RotationMatrixX(matrixRotationX, xRotation);
	double matrixRotationY[4*4];
	get4x4RotationMatrixY(matrixRotationY, yRotation);
	double matrixRotationZ[4*4];
	get4x4RotationMatrixZ(matrixRotationZ, zRotation);

	double temp[4*4];
	double temp1[4*4];

	getMultiplyMatrix(temp, matrixIdentity, 4, 4, matrixRotationX, 4, 4);
	getMultiplyMatrix(temp1, temp, 4, 4, matrixRotationY, 4, 4);
	getMultiplyMatrix(result, temp1, 4, 4, matrixRotationZ, 4, 4);


}

void SixAxisCalculations::get1x3PointMatrix(double result[], SixAxisCalculations::Point3D point){
	result[GET_INDEX(0, 0, 1)] = point.x;
	result[GET_INDEX(1, 0, 1)] = point.y;
	result[GET_INDEX(2, 0, 1)] = 1;
}

void SixAxisCalculations::get3x3RotationMatrix(double result[], double degrees){
	result[GET_INDEX(0, 0, 3)] = cos(degrees);
	result[GET_INDEX(0, 1, 3)] = -sin(degrees);
	result[GET_INDEX(0, 2, 3)] = 0;

	result[GET_INDEX(1, 0, 3)] = sin(degrees);
	result[GET_INDEX(1, 1, 3)] = cos(degrees);
	result[GET_INDEX(1, 2, 3)] = 0;

	result[GET_INDEX(2, 0, 3)] = 0;
	result[GET_INDEX(2, 1, 3)] = 0;
	result[GET_INDEX(2, 2, 3)] = 1;
}

void SixAxisCalculations::get3x3TransalationMatrix(double result[], double x, double y){
	result[GET_INDEX(0, 0, 3)] = 1;
	result[GET_INDEX(0, 1, 3)] = 0;
	result[GET_INDEX(0, 2, 3)] = x;

	result[GET_INDEX(1, 0, 3)] = 0;
	result[GET_INDEX(1, 1, 3)] = 1;
	result[GET_INDEX(1, 2, 3)] = y;

	result[GET_INDEX(2, 0, 3)] = 0;
	result[GET_INDEX(2, 1, 3)] = 0;
	result[GET_INDEX(2, 2, 3)] = 1;
}


void SixAxisCalculations::matrixTest(){



	double matrixA[] = MATRIX_IDENTITY_4X4;
	double matrixB[] = MATRIX_POINT_4X1;
	double result[] = MATRIX_EMPTY_4X1;



	getMultiplyMatrix(result, matrixA, 4, 4, matrixB, 4, 1);




	for(int i = 0; i < 4; i++){
		for(int x = 0; x < 1; x++){
			std::cout << result[GET_INDEX(i, x, 1)];
		}
		std::cout << std::endl;
	}


}


void SixAxisCalculations::getMultiplyMatrix(double result[], double matrixA[], int rowsA, int colsA, double matrixB[], int rowsB, int colsB){
	if (colsA != rowsB) {
			throw std::length_error("MatrixA:Columns did not match MatrixB:Rows!");
		}
		//vector< vector<double> > matrixC(rowsA, vector<double>(colsB));
		for (int i = 0; i < rowsA; i++) { // aRow
			//std::cout << "i:" << i << std::endl;
			for (int j = 0; j < colsB; j++) { // bColumn
				//std::cout << "j:" << j << std::endl;
				result[GET_INDEX(i, j, colsB)] = 0;
				for (int k = 0; k < colsA; k++) { // aColumn
					//std::cout << "k:" << k << std::endl;
					result[GET_INDEX(i, j, colsB)] += matrixA[GET_INDEX(i, k, colsA)] * matrixB[GET_INDEX(k,j, colsB)];
				}
			}
		}
		//return matrixC;
}












vector< vector<double> > SixAxisCalculations::multiplyMatrix(vector< vector<double> > matrixA, const int rowsA, const int colsA, vector< vector<double> > matrixB, const int rowsB, const int colsB){
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

double SixAxisCalculations::calculateAngle(double d2){
	return asin(d2/upperArmLength);
}

double SixAxisCalculations::calculateCircleIntersectionX(double CenterDistance, double radiusOne, double radiusTwo){
	return ((pow(CenterDistance, 2) - pow(radiusOne, 2) + pow(radiusTwo, 2)) / (2 * CenterDistance));
}

//Euclidean distance
double SixAxisCalculations::calculateCircleDistanceD(SixAxisCalculations::Point3D motorOrgin, SixAxisCalculations::Point3D effectorJointA){
	return sqrt(pow(motorOrgin.x - effectorJointA.x, 2) + pow(motorOrgin.y - effectorJointA.y, 2) +pow(motorOrgin.z - effectorJointA.z, 2));
}

double SixAxisCalculations::calculateAB(Point3D enginePosition, Point3D jointPosition){
	//double ac = sqrt(pow(enginePosition.x - jointPosition.x, 2) + pow(enginePosition.y - jointPosition.y, 2));
	double ac = enginePosition.x - jointPosition.x; //TODO was x
	return sqrt(pow(lowerArmLength, 2) + pow(ac, 2));
}

double SixAxisCalculations::getEffectorJointAngle(Point3D effectorJointPosition, Point3D neighbourEffectorJointPosition, double groupPositionOnCircle, double motorPositionOnCircle, double motorAngle){

	//Step 1: determinate the two vectors and transalte them so their intersection lies on the origin
	Point3D vectorEffector(	neighbourEffectorJointPosition.x - effectorJointPosition.x,
							neighbourEffectorJointPosition.y - effectorJointPosition.y,
							neighbourEffectorJointPosition.z - effectorJointPosition.z);

	/* The motors are positioned on the circle in a different angle than the effector joints.
	 * Get the right motor angles for the given effector joint angle.
	 */
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
	if(std::isnan(motorAngle) == true){
		return false;
	}


	double angle = abs(HALF_PI - getEffectorJointAngle(effectorJointPosition, neighbourEffectorJointPosition, groupPositionOnCircle, motorPositionOnCircle, motorAngle));
	//std::cout << "Critical angle: " << RADIANS_TO_DEGREES(angle) << std::endl;
	if(angle > maxJointAngle){
		return false;
	}
	return true;
}


double SixAxisCalculations::getAngleForMotor(Point3D moveTo, double groupPositionOnCircle, double effectorJointPositionOnCircle, double rotationX, double rotationY, double rotationZ){
	/**
	 * The six-axis math in four steps
	 *
	 * Step 1: Calculate the effector joint position for the given engine and based on the moveTo parameter which is the center of the effector.
	 * Step 2: 3d rotate the effector joint the given amount of degrees in the x, y and z plane around the center of the effector.
	 * Step 3: Translate the effector joint as if the engine would be at position 0, 0, 0.
	 * 		Step 3a: Rotate the joint (around the z axis) for the amount of degrees to motor would need to reach 0 degrees.
	 * 		Step 3b: Translate the joint the same amount the engine would need to reach x = 0.
	 * Step 4: Calculate the motor angle using inverse kinematics.
	 */



	/**
	 * Step 1: Effector joint position calculation
	 */

	double xJoint, yJoint;

	//Get the effector joint position based on the effector center and effector joint position on the circle
	xJoint = sin(DEGREES_TO_RADIANS(effectorJointPositionOnCircle)) * effectorJointtoCenterDistance;
	yJoint = cos(DEGREES_TO_RADIANS(effectorJointPositionOnCircle)) * effectorJointtoCenterDistance;



	//xJoint = getXEffectorPositionForEngineRotation(degreesToRadians(effectorJointPositionOnCircle));
	//yJoint = getYEffectorPositionForEngineRotation(degreesToRadians(effectorJointPositionOnCircle));


	/**
	 * Step 2: 3d rotation around the center of the effector in the x, y and z plane
	 */

	//Rotation will be calculated as if the effector center is in the origin of the coordinate system (Relative)
	Point3D moveToOrigin(xJoint, yJoint, 0);


	/*

	vector< vector<double> > matrixFull3DRotation = get4x4RotationMatrix(rotationX, rotationY, rotationZ);

	vector< vector<double> > pointMatrix = get1x4PointMatrix(moveToOrigin);

	vector< vector<double> > resultMatrix = multiplyMatrix(matrixFull3DRotation, 4, 4, pointMatrix, 4, 1);

	*/

	double matrixFull3DRotation[4*4];
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
	

	
	
	
	
	/**
	 * Step 3: Rotate around the z axis and translate the joint position as if the engine would be at position 0, 0, 0
	 */

	//Prepare matrixes


	/*
	vector< vector<double> >  matrixIdentity = get3x3IdentityMatrix();
	vector< vector<double> >  matrixRotation = get3x3RotationMatrix(degreesToRadians(groupPositionOnCircle));
	 */

	double matrixIdentity[] = MATRIX_IDENTITY_3X3;
	double matrixRotation[3*3];

	get3x3RotationMatrix(matrixRotation, DEGREES_TO_RADIANS(groupPositionOnCircle));


	//Calculate the needed translation on the x axis
	double xTrans = tan(DEGREES_TO_RADIANS((effectorJointPositionOnCircle -groupPositionOnCircle))) * motorAxisToCenterDistance;
	//double xTrans = tan(motorPositionOnCircle -motorGroupPositionOnCircle) * motorAxisToCenterDistance;

	/*
	vector< vector<double> >  matrixTransalation = get3x3TransalationMatrix(xTrans, -motorAxisToCenterDistance);


	//Calculate the complete rotation and translation matrix, and create the point matrix
	vector< vector<double> >  matrixRotationAndIdentity = multiplyMatrix(matrixIdentity, 3, 3, matrixRotation, 3, 3);
	vector< vector<double> >  calculationMatrix = multiplyMatrix(matrixTransalation, 3, 3, matrixRotationAndIdentity, 3, 3);

	vector< vector<double> >  rotatedPointMatrix = get1x3PointMatrix(rotatedPoint);


	//Calculate the translated and rotated position of the 3d rotated point from step 1
	vector< vector<double> >  resultPoint = multiplyMatrix(calculationMatrix, 3, 3, rotatedPointMatrix, 3, 1);
	*/


	double matrixTransalation[3*3];
	get3x3TransalationMatrix(matrixTransalation, xTrans, -motorAxisToCenterDistance);


	//Calculate the complete rotation and translation matrix, and create the point matrix
	double matrixRotationAndIdentity[3*3];
	getMultiplyMatrix(matrixRotationAndIdentity, matrixIdentity, 3, 3, matrixRotation, 3, 3);

	double calculationMatrix[3*3];
	getMultiplyMatrix(calculationMatrix, matrixTransalation, 3, 3, matrixRotationAndIdentity, 3, 3);

	double rotatedPointMatrix[3*1];
	get1x3PointMatrix(rotatedPointMatrix, rotatedPoint);


	//Calculate the translated and rotated position of the 3d rotated point from step 1
	double resultPoint[3*1];
	getMultiplyMatrix(resultPoint, calculationMatrix, 3, 3, rotatedPointMatrix, 3, 1);










	SixAxisCalculations::Point3D moveToTransalated( resultPoint[GET_INDEX(0, 0, 1)], resultPoint[GET_INDEX(1, 0, 1)], rotatedPoint.z);






	SixAxisCalculations::Point3D engine(0, 0, 0);

	/**
	 * Step 4: Inverse kinematics.
	 */

	double ab = calculateAB(engine,  moveToTransalated);
	double d = calculateCircleDistanceD(engine, moveToTransalated);
	double xd = calculateCircleIntersectionX(d, upperArmLength, ab);
	double d2 = d - xd;

	return calculateAngle(d2);
}

double SixAxisCalculations::getAngleBetween(Point3D vectorOne, Point3D vectorTwo){
	//Calculate the angle between the two vectors: http://en.wikipedia.org/wiki/Dot_product
	double dotProduct = (vectorOne.x * vectorTwo.x) + (vectorOne.y * vectorTwo.y) + (vectorOne.z * vectorTwo.z);

	double magnitudeVectorOne = sqrt(pow(vectorOne.x, 2) + pow(vectorOne.y, 2) + pow(vectorOne.z, 2));
	double magnitudeVectorTwo = sqrt(pow(vectorTwo.x, 2) + pow(vectorTwo.y, 2) + pow(vectorTwo.z, 2));

	return acos(dotProduct / (magnitudeVectorOne * magnitudeVectorTwo));
}

SixAxisCalculations::EffectorMove SixAxisCalculations::getMotorAngles(Point3D moveTo, double xRotation, double yRotation, double zRotation){

	EffectorMove result;

	result.moveTo = moveTo;
	result.effectorRotationX = xRotation;
	result.effectorRotationY = yRotation;
	result.effectorRotationZ = zRotation;
	result.validMove = true;


	//Group A
	result.angles[0] = getAngleForMotor(moveTo, SixAxisCalculations::GROUP_A_POS, SixAxisCalculations::EFFECTOR_JOINT_A1_POS, xRotation, yRotation, zRotation);
	result.angles[1] = getAngleForMotor(moveTo, SixAxisCalculations::GROUP_A_POS, SixAxisCalculations::EFFECTOR_JOINT_A2_POS, xRotation, yRotation, zRotation);

	if(!isValidPosition(effectorJointPositionCache[0], effectorJointPositionCache[1], result.angles[0], SixAxisCalculations::GROUP_A_POS, SixAxisCalculations::MOTOR_A1_POS)){
		result.validMove = false;
		result.angles[0] = NAN;
	}
	if(!isValidPosition(effectorJointPositionCache[1], effectorJointPositionCache[0], result.angles[1], SixAxisCalculations::GROUP_A_POS, SixAxisCalculations::MOTOR_A2_POS)){
		result.validMove = false;
		result.angles[1] = NAN;
	}


	//Group B
	result.angles[2] = getAngleForMotor(moveTo, SixAxisCalculations::GROUP_B_POS, SixAxisCalculations::EFFECTOR_JOINT_B1_POS, xRotation, yRotation, zRotation);
	result.angles[3] = getAngleForMotor(moveTo, SixAxisCalculations::GROUP_B_POS, SixAxisCalculations::EFFECTOR_JOINT_B2_POS, xRotation, yRotation, zRotation);

	if(!isValidPosition(effectorJointPositionCache[2], effectorJointPositionCache[3], result.angles[2], SixAxisCalculations::GROUP_B_POS, SixAxisCalculations::MOTOR_B1_POS)){
		result.validMove = false;
		result.angles[2] = NAN;
	}
	if(!isValidPosition(effectorJointPositionCache[3], effectorJointPositionCache[2], result.angles[3], SixAxisCalculations::GROUP_B_POS, SixAxisCalculations::MOTOR_B2_POS)){
		result.validMove = false;
		result.angles[3] = NAN;
	}


	//Group C
	result.angles[4] = getAngleForMotor(moveTo, SixAxisCalculations::GROUP_C_POS, SixAxisCalculations::EFFECTOR_JOINT_C1_POS, xRotation, yRotation, zRotation);
	result.angles[5] = getAngleForMotor(moveTo, SixAxisCalculations::GROUP_C_POS, SixAxisCalculations::EFFECTOR_JOINT_C2_POS, xRotation, yRotation, zRotation);

	if(!isValidPosition(effectorJointPositionCache[4], effectorJointPositionCache[5], result.angles[4], SixAxisCalculations::GROUP_C_POS, SixAxisCalculations::MOTOR_C1_POS)){
		result.validMove = false;
		result.angles[4] = NAN;
	}
	if(!isValidPosition(effectorJointPositionCache[5], effectorJointPositionCache[4], result.angles[5], SixAxisCalculations::GROUP_C_POS, SixAxisCalculations::MOTOR_C2_POS)){
		result.validMove = false;
		result.angles[5] = NAN;
	}
	return result;
}

double * SixAxisCalculations::getAngles(double angles[6], Point3D moveTo, double xRotation, double yRotation, double zRotation){
	EffectorMove move = getMotorAngles(moveTo, xRotation, yRotation, zRotation);
	if(move.validMove){
		angles[0] = move.angles[0];
		angles[1] = move.angles[1];
		angles[2] = move.angles[2];
		angles[3] = move.angles[3];
		angles[4] = move.angles[4];
		angles[5] = move.angles[5];
	} else {
		angles[0] = NAN;
		angles[1] = NAN;
		angles[2] = NAN;
		angles[3] = NAN;
		angles[4] = NAN;
		angles[5] = NAN;
	}
	return angles;
}

bool SixAxisCalculations::isValidMove(double angles[6]){
	for(int i = 0; i < 6;i++){
		if(std::isnan(angles[i]) == 1){
			return false;
		}
	}
	return true;
}



bool SixAxisCalculations::checkPath(Point3D from, double startRotationX, double startRotationY, double startRotationZ, Point3D to, double endRotationX, double endRotationY, double endRotationZ){
	
		std::cout << "path check start" << std::endl;
    	std::cout << "fromx: " << from.x << " fromy: " << from.y << " fromz: " << from.z << std::endl;
		std::cout << "tox: " << to.x << " toy: " << to.y << " toz: " << to.z << std::endl;

		//Calculate lengths to travel
    	double x_length = to.x - from.x;
    	double y_length = to.y - from.y;
    	double z_length = to.z - from.z;
		
		
		
    	double largest_length = (double)	(fabs(x_length) > fabs(y_length) ?
												(fabs(x_length) > fabs(z_length) ? fabs(x_length) : fabs(z_length)) :
												(fabs(y_length) > fabs(z_length) ? fabs(y_length) : fabs(z_length))	);

    	double stepLengthX = x_length / largest_length;
    	double stepLengthY = y_length / largest_length;
    	double stepLengthZ = z_length / largest_length;
 
		std::cout << "path length: " << largest_length << std::endl;
		
		for(double step = 1; step <= largest_length; step++){
			
			double x = (from.x + stepLengthX * step);
			double y = (from.y + stepLengthY * step);
			double z = (from.z + stepLengthZ * step);
			
			//std::cout << "checking coordinate: " << x << ", " << y << ", " << z << std::endl;
			
			////BitmapCoordinate temp = fromRealCoordinate(rexos_datatypes::Point3D<double>(x, y, z));
			//int index = temp.x + temp.y * width + temp.z * width * depth;

			//if(temp.x < 0 || temp.y < 0 || temp.z < 0) {
				//std::cout << "temp.x: " << temp.x << " or temp.y: " << temp.y << std::endl;
				//return false;
			//}

			//if(temp.x > width){
				//std::cout << "temp.x: " << temp.x << " > width: " << width << std::endl;
			//	return false;
			//}
			//if(temp.y > depth){
				//std::cout << "temp.y: " << temp.y << " > depth: " << depth << std::endl;
				//return false;
			//}
			//if(temp.z > height){
				//std::cout << "temp.z: " << temp.z << " > height: " << height << std::endl;
				//return false;
			//}
			
			EffectorMove move = getMotorAngles(Point3D(x, y, z), 0, 0, 0);
			if(!move.validMove){
				std::cout << "invalid position in path: " << move.moveTo << std::endl;
				return false;
			}
			
			
			//if(boundariesBitmap[index] != VALID){
				//std::cout << "boundariesBitmap[index]: " << boundariesBitmap[index] << "!= VALID: " << VALID << std::endl;
				//return false;
			//}
		}
        return true;
	
}






