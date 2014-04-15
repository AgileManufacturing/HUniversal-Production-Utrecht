/*
 * SixAxisMath.cpp
 *  Copyright: Rolf Smit
 *  Created on: 4 apr. 2014
 *      Author: Rolf
 */
  
#include "rexos_stewart_gough/SixAxisCalculations.h"
using namespace std;
  
  namespace rexos_stewart_gough{
  
vector< vector<double> > SixAxisCalculations::get3DIdentityMatrix(){
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
  
  
vector< vector<double> > SixAxisCalculations::get3DTransalationMatrix(Point3D point){
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
  
  
  
  
vector< vector<double> > SixAxisCalculations::get3DRotationMatrixX(double xRotation){
    double xRadians = degreesToRadians(xRotation);
  
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
vector< vector<double> > SixAxisCalculations::get3DRotationMatrixY(double yRotation){
    double yRadians = degreesToRadians(yRotation);
  
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
vector< vector<double> > SixAxisCalculations::get3DRotationMatrixZ(double zRotation){
    double zRadians = degreesToRadians(zRotation);
  
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
  
  
vector< vector<double> > SixAxisCalculations::get3DPointMatrix(Point3D point){
    vector< vector<double> > matrix(4, vector<double>(1));
    matrix[0][0] = point.x;
    matrix[1][0] = point.y;
    matrix[2][0] = point.z;
    matrix[3][0] = 1;
    return matrix;
}
  
vector< vector<double> > SixAxisCalculations::get3DRotationMatrix(double xRotation, double yRotation, double zRotation){
  
    vector< vector<double> > matrixIdentity = get3DIdentityMatrix();
  
  
    vector< vector<double> > matrixRotationX = get3DRotationMatrixX(xRotation);
    vector< vector<double> > matrixRotationY = get3DRotationMatrixY(yRotation);
    vector< vector<double> > matrixRotationZ = get3DRotationMatrixZ(zRotation);
  
  
    vector< vector<double> >  temp;
  
    temp = multiplyMatrix(matrixIdentity, 4, 4, matrixRotationX, 4, 4);
    temp = multiplyMatrix(temp, 4, 4, matrixRotationY, 4, 4);
    temp = multiplyMatrix(temp, 4, 4, matrixRotationZ, 4, 4);
  
  
    //return matrixRotationZ;
    return temp;
}
  
vector< vector<double> > SixAxisCalculations::getIdentityMatrix(){
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
  
vector< vector<double> > SixAxisCalculations::getPointMatrix(SixAxisCalculations::Point3D point){
    vector< vector<double> > matrix(3, vector<double>(1));
    matrix[0][0] = point.x;
    matrix[1][0] = point.y;
    matrix[2][0] = 1;
    return matrix;
}
  
vector< vector<double> > SixAxisCalculations::getRotationMatrix(double degrees){
    double radians = degreesToRadians(degrees);
  
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
  
vector< vector<double> > SixAxisCalculations::getTransalationMatrix(double x, double y){
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
    return radiansToDegrees((asin(d2/upperArmLength)));
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
  
  
  
  
  
  
  
  
  
//double  SixAxisMath::getXEnginePositionForEngineRotation(double rotation){
    //return sin((degreesToRadians(rotation))) * motorAxisToCenterDistance;
//}
//double  SixAxisMath::getYEnginePositionForEngineRotation(double rotation){
    //return cos((degreesToRadians(rotation))) * motorAxisToCenterDistance;
//}
  
  
  
  
double  SixAxisCalculations::getXEffectorPositionForEngineRotation(double rotation){
    return sin((degreesToRadians(rotation))) * effectorJointtoCenterDistance;
}
double  SixAxisCalculations::getYEffectorPositionForEngineRotation(double rotation){
    return cos((degreesToRadians(rotation))) * effectorJointtoCenterDistance;
}
  
  
  
double SixAxisCalculations::getAngleForMotor(Point3D moveTo, double motorPositionOnCircle){
  
    double x, y;
  
    //if(motorPositionOnCircle != 0){
        //Prepare normalization matrixes
        vector< vector<double> > matrixIdentity = getIdentityMatrix();
  
        vector< vector<double> >  matrixRotation = getRotationMatrix(motorPositionOnCircle);
  
        vector< vector<double> >  matrixTransalation = getTransalationMatrix(-motorAxisToCenterDistance, 0);
  
        vector< vector<double> >  matrixRotationAndIdentity = multiplyMatrix(matrixIdentity, 3, 3, matrixRotation, 3, 3);
        vector< vector<double> >  calculationMatrix = multiplyMatrix(matrixTransalation, 3, 3, matrixRotationAndIdentity, 3, 3);
  
        vector< vector<double> >  pointMatrix = getPointMatrix(moveTo);
        vector< vector<double> >  resultPoint = multiplyMatrix(calculationMatrix, 3, 3, pointMatrix, 3, 1);
  
        x = resultPoint[0][0];
        y = resultPoint[1][0];
    //} else {
    //  x = moveTo.x - motorAxisToCenterDistance;
        //y = moveTo.y;
    //}
  
    SixAxisCalculations::Point3D engine(0, 0, 0);
    SixAxisCalculations::Point3D moveToTransalated(x, y, moveTo.z);
  
    double ab = calculateAB(engine,  moveToTransalated);
  
    double d = calculateCircleDistanceD(engine, moveToTransalated);
  
  
    double xd = calculateCircleIntersectionX(d, upperArmLength, ab);
  
    double d2 = d - xd;
  
    return calculateAngle(d2);
}
  
  
  
double SixAxisCalculations::getAngleForMotor(Point3D moveTo, double motorGroupPositionOnCircle, double motorPositionOnCircle, double rotationX, double rotationY, double rotationZ){
    /**
     * The six-axis math in four steps
     *
     * Step 1: Calculate the effector joint position for the given engine and based on the moveTo parameter which is the center of the effector.
     * Step 2: 3d rotate the effector joint the given amount of degrees in the x, y and z plane around the center of the effector.
     * Step 3: Translate the effector joint as if the engine would be at position 0, 0, 0.
     *      Step 3a: Rotate the joint (around the z axis) for the amount of degrees to motor would need to reach 0 degrees.
     *      Step 3b: Translate the joint the same amount the engine would need to reach x = 0.
     * Step 4: Calculate the motor angle using inverse kinematics.
     */
  
  
  
    /**
     * Step 1: Effector joint position calculation
     */
  
    double xJoint, yJoint;
  
    xJoint = getXEffectorPositionForEngineRotation(motorPositionOnCircle);
    yJoint = getYEffectorPositionForEngineRotation(motorPositionOnCircle);
  
  
  
  
  
    /**
     * Step 2: 3d rotation around the center of the effector in the x, y and z plane
     */
  
    //Rotation will be calculated as if the effector center is in the origin of the coordinate system (Relative)
    Point3D moveToOrigin(xJoint, yJoint, 0);
  
    vector< vector<double> > matrixFull3DRotation = get3DRotationMatrix(rotationX, rotationY, rotationZ);
  
    vector< vector<double> > pointMatrix = get3DPointMatrix(moveToOrigin);
  
    vector< vector<double> > resultMatrix = multiplyMatrix(matrixFull3DRotation, 4, 4, pointMatrix, 4, 1);
  
    //Construct the rotated point and transalate it to the real position in the coordinate system (Absolute)
    Point3D rotatedPoint(resultMatrix[0][0] + moveTo.x, resultMatrix[1][0] + moveTo.y, resultMatrix[2][0] + moveTo.z);
  
    /**
     * Step 3: Rotate around the z axis and translate the joint position as if the engine would be at position 0, 0, 0
     */
  
    //Prepare matrixes
    vector< vector<double> >  matrixIdentity = getIdentityMatrix();
    vector< vector<double> >  matrixRotation = getRotationMatrix(motorGroupPositionOnCircle);
  
  
    //Calculate the needed translation on the x axis
    double xTrans = tan(motorPositionOnCircle -motorGroupPositionOnCircle) * motorAxisToCenterDistance;
  
  
    vector< vector<double> >  matrixTransalation = getTransalationMatrix(xTrans, -motorAxisToCenterDistance);
  
    //Calculate the complete rotation and translation matrix, and create the point matrix
    vector< vector<double> >  matrixRotationAndIdentity = multiplyMatrix(matrixIdentity, 3, 3, matrixRotation, 3, 3);
    vector< vector<double> >  calculationMatrix = multiplyMatrix(matrixTransalation, 3, 3, matrixRotationAndIdentity, 3, 3);
  
    vector< vector<double> >  rotatedPointMatrix = getPointMatrix(rotatedPoint);
  
    //Calculate the translated and rotated position of the 3d rotated point from step 1
    vector< vector<double> >  resultPoint = multiplyMatrix(calculationMatrix, 3, 3, rotatedPointMatrix, 3, 1);
  
    SixAxisCalculations::Point3D moveToTransalated( resultPoint[0][0], resultPoint[1][0], rotatedPoint.z);
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
  
  
double * SixAxisCalculations::getAngles(double angles[6], Point3D moveTo, double xRotation, double yRotation, double zRotation){
  
    angles[0] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_A_POS_ON_CIRCLE, SixAxisCalculations::MOTOR_A1_POS_ON_CIRCLE, xRotation, yRotation, zRotation);
    angles[1] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_A_POS_ON_CIRCLE, SixAxisCalculations::MOTOR_A2_POS_ON_CIRCLE, xRotation, yRotation, zRotation);
  
    angles[2] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_B_POS_ON_CIRCLE, SixAxisCalculations::MOTOR_B1_POS_ON_CIRCLE, xRotation, yRotation, zRotation);
    angles[3] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_B_POS_ON_CIRCLE, SixAxisCalculations::MOTOR_B2_POS_ON_CIRCLE, xRotation, yRotation, zRotation);
  
    angles[4] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_C_POS_ON_CIRCLE, SixAxisCalculations::MOTOR_C2_POS_ON_CIRCLE, xRotation, yRotation, zRotation);
    angles[5] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_C_POS_ON_CIRCLE, SixAxisCalculations::MOTOR_C1_POS_ON_CIRCLE, xRotation, yRotation, zRotation);
  
	for(int i = 0; i < 6; i++){
		angles[i] = degreesToRadians(angles[i]);
	}
  
    return angles;
}
  
double * SixAxisCalculations::getAngles(double angles[6], Point3D moveTo){
  
    angles[0] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_A_POS_ON_CIRCLE);
	angles[1] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_A_POS_ON_CIRCLE);
    angles[2] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_B_POS_ON_CIRCLE);
	angles[3] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_B_POS_ON_CIRCLE);
	angles[4] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_C_POS_ON_CIRCLE);
	angles[5] = getAngleForMotor(moveTo, SixAxisCalculations::MOTOR_C_POS_ON_CIRCLE);
	
	
	for(int i = 0; i < 6; i++){
	    angles[i] = degreesToRadians(angles[i]);
	}
  
    return angles;
}
  
bool SixAxisCalculations::isValidMove(double angles[6]){
	double nan = 0.0/0.0;
	//isnan
    for(int i = 0; i < 6;i++){
        if(nan == angles[i]){
            return false;
        }
    }
  
  
    return true;
}
  }