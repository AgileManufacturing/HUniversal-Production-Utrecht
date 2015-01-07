/*
 * SixAxisMath.cpp
 *
 *  Created on: 4 apr. 2014
 *      Author: Rolf
 */

#include "stewart_gough_simulation_node/SixAxisMath.h"
using namespace std;



double SixAxisMath::getAngleForMotor(Point3D moveTo, double motorPositionOnCircle){

	//cout << endl << endl << endl;
	//cout << "Starting calculations:" << endl;

	//Transalated "moveTo" point
	double x, y;

	if(motorPositionOnCircle != 0){

		//cout << "Starting matrix transalations" << endl;


		//Prepare normalization matrixes
		vector< vector<double> > matrixIdentity = getIdentityMatrix();


		vector< vector<double> >  matrixRotation = getRotationMatrix(motorPositionOnCircle);


		vector< vector<double> >  matrixTransalation = getTransalationMatrix(-motorAxisToCenterDistance, 0);



		vector< vector<double> >  matrixRotationAndIdentity = multiplyMatrix(matrixIdentity, 3, 3, matrixRotation, 3, 3);
		vector< vector<double> >  calculationMatrix = multiplyMatrix(matrixTransalation, 3, 3, matrixRotationAndIdentity, 3, 3);



		vector< vector<double> >  pointMatrix = getPointMatrix(moveTo);
		vector< vector<double> >  resultPoint = multiplyMatrix(calculationMatrix, 3, 3, pointMatrix, 3, 1);



		//deleteMatrixArray(matrixRotation, 3, 3);
		//deleteMatrixArray(matrixTransalation, 3, 3);
		//deleteMatrixArray(matrixRotationAndIdentity, 3, 3);
		//deleteMatrixArray(calculationMatrix, 3, 3);
		//deleteMatrixArray(pointMatrix, 3, 1);


		x = resultPoint[0][0];
		y = resultPoint[1][0];
		//deleteMatrixArray(resultPoint, 3, 1);

	} else {
		//cout << "Skipping matrix transalations, not needed" << endl;

		x = moveTo.x - motorAxisToCenterDistance;
		y = moveTo.y;
	}






			//x = x - 11.0f;
			//y = y - 0.0f;

			//if(!verbose){
	//cout << "Transalated:" << endl;
	//cout << "x: " << x << "  y: " << y << endl;
				//System.out.println("x: " + x + "  y: " + y);
			//}




			SixAxisMath::Point3D engine(0, 0, 0);
			SixAxisMath::Point3D moveToTransalated(x, y, moveTo.z);

			double ab = calculateAB(engine,  moveToTransalated);



		//	cout << "ab: " << ab << endl;
			//if(!verbose){
				//System.out.println("Calculated AB: " + ab);
			//}

			//cout << "z1: " << engine.z << endl;
			//cout << "z2: " << moveToTransalated.z << endl;


			//double centerDistance = (engine.z - moveToTransalated.z);

			//cout << "centerDistance: " << centerDistance << endl;

			double xd = calculateCircleIntersectionX(std::abs((double) (engine.z - moveToTransalated.z)), upperArmLength, ab);

		//	cout << "xd: " << xd << endl;

			//if(!verbose){
			//	System.out.println("Calculated x: " + xd);
			//}

			double d = calculateCircleDistanceD(engine, moveToTransalated);

			//cout << "d: " << d << endl;
			//if(!verbose){
			//	System.out.println("Calculated d: " + d);
			//}

			double d2 = d - xd;

			//cout << "d2: " << d2 << endl;
			//if(!verbose){
			//	System.out.println("Calculated d2: " + d2);
			//}

			double calculatedAngle = calculateAngle(d2);
			//if(!verbose){
			//	System.out.println("Calculated angle: " + calculatedAngle);
			//}
			return calculatedAngle;





	//return NAN; //NaN (Not-A-Number)
}




vector< vector<double> > SixAxisMath::getIdentityMatrix(){
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

vector< vector<double> > SixAxisMath::getPointMatrix(SixAxisMath::Point3D point){
	vector< vector<double> > matrix(3, vector<double>(1));
	matrix[0][0] = point.x;
	matrix[1][0] = point.y;
	matrix[2][0] = 1;
	return matrix;
}

vector< vector<double> > SixAxisMath::getRotationMatrix(double degrees){
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

vector< vector<double> > SixAxisMath::getTransalationMatrix(double x, double y){
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

void SixAxisMath::deleteMatrixArray(double** matrix, int rows, int cols){
	int i;
	for(i = 0; i < rows; i++){
		delete[] matrix[i];
	}
	delete[] matrix;
}



vector< vector<double> > SixAxisMath::multiplyMatrix(vector< vector<double> > matrixA, const int rowsA, const int colsA, vector< vector<double> > matrixB, const int rowsB, const int colsB){


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




double SixAxisMath::calculateAngle(double d2){
	return radiansToDegrees((asin(d2/upperArmLength)));
}

double SixAxisMath::calculateCircleIntersectionX(double CenterDistance, double radiusOne, double radiusTwo){
	return ((pow(CenterDistance, 2) - pow(radiusOne, 2) + pow(radiusTwo, 2)) / (2 * CenterDistance));
}

//Euclidean distance
double SixAxisMath::calculateCircleDistanceD(SixAxisMath::Point3D motorOrgin, SixAxisMath::Point3D effectorJointA){
	return sqrt(pow(motorOrgin.x - effectorJointA.x, 2) + pow(motorOrgin.y - effectorJointA.y, 2) +pow(motorOrgin.z - effectorJointA.z, 2));
}

double SixAxisMath::calculateAB(Point3D enginePosition, Point3D jointPosition){
	double ac = enginePosition.y - jointPosition.y; //TODO was x
	return sqrt(pow(lowerArmLength, 2) + pow(ac, 2));
}
