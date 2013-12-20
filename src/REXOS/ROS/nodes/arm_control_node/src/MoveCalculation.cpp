/**
 * @file SixAxis.h
 * @brief class to control the SixAxis' movements
 * @date Created: 24-10-2013
 *
 * @author Alexander Hustinx
 *
 * @section LICENSE
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/
#include "ros/ros.h"
#include "arm_control_node/MoveCalculations.h"
#include <math.h>
#include <stdio.h>      /* printf */
#include <stdlib.h>     /* abs */
#include <iostream>

#define M_PI	3.14159265358979323846

MoveCalculations::MoveCalculations() {

	// START:	Machine defined values
	basisPositionBallJoints[0] = 0;				//Position of ball-joint 1 on the basis (in degrees)
	basisPositionBallJoints[1] = 120;			//Position of ball-joint 2 on the basis (in degrees)
	basisPositionBallJoints[2] = 120;			//Position of ball-joint 3 on the basis (in degrees)
	basisPositionBallJoints[3] = 240;			//Position of ball-joint 4 on the basis (in degrees)
	basisPositionBallJoints[4] = 240;			//Position of ball-joint 5 on the basis (in degrees)
	basisPositionBallJoints[5] = 0;				//Position of ball-joint 6 on the basis (in degrees)

	distancesBallJointsAndArms[0] = 31.28;		//Distance between balljoint and arm (in mm?)
	distancesBallJointsAndArms[1] = -31.28;		//Distance between balljoint and arm (in mm?)
	distancesBallJointsAndArms[2] = 31.28;		//Distance between balljoint and arm (in mm?)
	distancesBallJointsAndArms[3] = -31.28;		//Distance between balljoint and arm (in mm?)
	distancesBallJointsAndArms[4] = 31.28;		//Distance between balljoint and arm (in mm?)
	distancesBallJointsAndArms[5] = -31.28;		//Distance between balljoint and arm (in mm?)


	radiusBasis = 100;							//Radius Basis (in mm)
	radiusEffector = 40;						//Radius Effector (in mm)
	toolCenterPoint = 0;						//Tool Center Point (in mm)

	//Actuator setup:
	minAngle = -20;								//minimum Angle	(in degrees)
	maxAngle = 90;								//maximum Angle (in degrees)

	//Parameters arm:
	lengthUpperArm = 100;						//length upper arm (in mm)
	lengthLowerArm = 350;						//length lower arm (in mm)

	ballJointRestriction = 16;					//Ball-joints restrictions (in degrees)
	// END:		Machine defined values
}

MoveCalculations::MoveCalculations(double gammaB1, double gammaB2, double gammaB3, double gammaB4, double gammaB5, double gammaB6, double eB1, double eB2, double eB3, double eB4, double eB5, double eB6, int rb, int tCP, int minimumAngle, int maximumAngle, int uArmLength, int lArmLength, int ballJointsRestrictions, int rp)
{
	basisPositionBallJoints[0] = gammaB1;
	basisPositionBallJoints[1] = gammaB2;
	basisPositionBallJoints[2] = gammaB3;
	basisPositionBallJoints[3] = gammaB4;
	basisPositionBallJoints[4] = gammaB5;
	basisPositionBallJoints[5] = gammaB6;

	distancesBallJointsAndArms[0] = eB1;
	distancesBallJointsAndArms[1] = eB2;
	distancesBallJointsAndArms[2] = eB3;
	distancesBallJointsAndArms[3] = eB4;
	distancesBallJointsAndArms[4] = eB5;
	distancesBallJointsAndArms[5] = eB6;

	radiusBasis = rb, toolCenterPoint = tCP, minAngle = minimumAngle, maxAngle = maximumAngle, lengthUpperArm = uArmLength, lengthLowerArm = lArmLength, ballJointRestriction = ballJointsRestrictions, radiusEffector = rp;
}

bool MoveCalculations::moveEffectorToPoint(double x, double y, double z, double rX, double rY, double rZ){
	//	TODO:	Actually move the motor ... 
	
	for(int i = 0; i < 6; i++){
		if((angles[i] = calcMotorAngle(x, y, z, rX, rY, rZ, basisPositionBallJoints[i], distancesBallJointsAndArms[i])) == -360){
			//return false;
		}
	}
	/*
	ros::NodeHandle nodeHandle;
	ros::ServiceClient client = nodeHandle.serviceClient<simulation_node::rotate>("rotate_motor_to_degrees");
	simulation_node::rotate rotate;
	
	
	for(int i = 1; i <= 6; i++) {
	//	TODO:	Actually move the motor to angles[i]
		rotate.request.motorIndex = i-1;
		rotate.request.degrees = angles[i-1]*-1;
		if (client.call(rotate)){
			ROS_INFO("PRESSED W");
		}else{
			ROS_ERROR("Failed to call service");
		}
		std::cout << "\tMoving Motor #" << i << " to angle: " << angles[i-1] << "\n";
	}
	std::cout << "\n";
*/
	//update last angles
	for(int i = 0; i < 6; i++){
		lastAngles[i] = angles[i];
	}

	//update last coords
	lastX = x;
	lastY = y;
	lastZ = z;
	lastRX = rX;
	lastRY = rY;
	lastRZ = rZ;

	return true;
}


bool MoveCalculations::moveEffectorToRelativePoint(double x, double y, double z, double rX, double rY, double rZ){
	//	TODO:	Actually move the motor ... 
	
	for(int i = 0; i < 6; i++){
		if((angles[i] = calcMotorAngle(x, y, z, rX, rY, rZ, basisPositionBallJoints[i], distancesBallJointsAndArms[i])) == -360){
			return false;
		}
	}
	
	
	for(int i = 0; i < 6; i++) {
	//	TODO:	Actually move the motor to angles[i]
		angles[i] = angles[i] - lastAngles[i];
		if(angles[i] > 360)		angles[i] = angles[i] - 360;
		if(angles[i] < -360)	angles[i] = angles[i] + 360;
		
		std::cout << "\tMoving Motor #" << i+1 << " to angle: " << angles[i] << "\n";
	}
	std::cout << "\n";

	//update last angles
	for(int i = 0; i < 6; i++){
		lastAngles[i] = angles[i];
	}

	//update last coords
	lastX = x;
	lastY = y;
	lastZ = z;
	lastRX = rX;
	lastRY = rY;
	lastRZ = rZ;

	return true;
}


double MoveCalculations::calcMotorAngle(double x, double y, double z, double rX, double rY, double rZ, double gammaB, double eP){

	//Calcs:
	double desiredRxRadians = rX * M_PI / 180;	//Rx (in rads)
	double desiredRyRadians = rY * M_PI / 180;	//Ry (in rads)
	double desiredRzRadians = rZ * M_PI / 180;	//Rz (in rads)

	/*	basisPositionBallJoint generic	*/
	double basisPositionBallJointRadians = gammaB * M_PI / 180;		//Gamma B1 (in rads)

	//Calcs:	qi Est. Angle:
	double estimatedAngleDegrees = 0;			//Estimated angle (in degrees)
	double upperBounds = maxAngle - minAngle;	//Bounds upper (in degrees)
	double lowerBounds = 0;						//Bounds lower (in degrees)
	bool isTooFar = false;						//Checks too far: 1 or 0

	int iterations = 15;						//Amount of iterations

	
	double lowerArmX, lowerArmY, lowerArmZ;
	double upperArmX, upperArmY, upperArmZ;
	double effectorX, effectorY, effectorZ;

	double newEstimatedAngleDegrees;
	double lowerArmEstematedLength;

	for( int i = 0; i < iterations; i++ ) {
		if( i == 0);		
		else if(i == 1){
			estimatedAngleDegrees = maxAngle - minAngle;
		} else {
			estimatedAngleDegrees = (upperBounds + lowerBounds) / 2;
		}
		newEstimatedAngleDegrees = estimatedAngleDegrees + minAngle;				//Estimated angle + minimum angle (in degrees)
		double newEstimatedAngleRadians = newEstimatedAngleDegrees * M_PI / 180;	//Estimated angle + minimum angle (in rads)

		//Calcs:	Vector upper arm:
		upperArmX = cos(basisPositionBallJointRadians) * (cos(newEstimatedAngleRadians) * lengthUpperArm + radiusBasis)						//X (in mm)
			- sin(basisPositionBallJointRadians) * eP;	
		upperArmY = sin(basisPositionBallJointRadians) * (cos(newEstimatedAngleRadians) * lengthUpperArm + radiusBasis)						//Y (in mm)
			+ cos(basisPositionBallJointRadians) * eP;	
		upperArmZ = -sin(newEstimatedAngleRadians) * lengthUpperArm;																		//Z (in mm)

		//Calcs:	Vector effector:
		effectorX = ((cos(desiredRzRadians) * cos(desiredRyRadians) - sin(desiredRzRadians) * sin(desiredRxRadians)							//X (in mm)
			* sin(desiredRyRadians)) * (cos(basisPositionBallJointRadians)	* radiusEffector - sin(basisPositionBallJointRadians) 
			* eP) - sin(desiredRzRadians) * cos(desiredRxRadians) * (sin(basisPositionBallJointRadians) 
			* radiusEffector + cos(basisPositionBallJointRadians) * eP) + (cos(desiredRzRadians) 
			* sin(desiredRyRadians) + sin(desiredRzRadians) * sin(desiredRxRadians) * cos(desiredRyRadians)) * toolCenterPoint) + x;

		effectorY = ((sin(desiredRzRadians) * cos(desiredRyRadians) + cos(desiredRzRadians) * sin(desiredRxRadians)							//Y (in mm)
			* sin(desiredRyRadians)) * (cos(basisPositionBallJointRadians) * radiusEffector - sin(basisPositionBallJointRadians) 
			* eP) + cos(desiredRzRadians) * cos(desiredRxRadians) * (sin(basisPositionBallJointRadians) 
			* radiusEffector + cos(basisPositionBallJointRadians) * eP) + (sin(desiredRzRadians) 
			* sin(desiredRyRadians) - cos(desiredRzRadians) * sin(desiredRxRadians) * cos(desiredRyRadians)) * -toolCenterPoint) + y;

		effectorZ = z + eP * cos(basisPositionBallJointRadians) * sin(desiredRxRadians)	+ radiusEffector 									//Z (in mm)
			* sin(basisPositionBallJointRadians) * sin(desiredRxRadians) - cos(desiredRxRadians) * (toolCenterPoint 
			* cos(desiredRyRadians) + (radiusEffector * cos(basisPositionBallJointRadians) - eP 
			* sin(basisPositionBallJointRadians)) * sin(desiredRyRadians));

		//Calcs:	Vector lower arm Ri:
		lowerArmX = upperArmX - effectorX;		//X (in mm)
		lowerArmY = upperArmY - effectorY;		//Y (in mm)
		lowerArmZ = upperArmZ - effectorZ;		//Z (in mm)

		//Calcs:	Lower arm specs:
		lowerArmEstematedLength = sqrt(lowerArmX*lowerArmX + lowerArmY*lowerArmY + lowerArmZ*lowerArmZ);	//Estimated length (in mm)

		if(lowerArmEstematedLength > lengthLowerArm){
			isTooFar = true;
		} else {
			isTooFar = false;
		}

		if(i == 1){
			upperBounds = maxAngle - minAngle;
			lowerBounds = 0; 
		} else {

			if(isTooFar == 0){
				upperBounds = estimatedAngleDegrees;									
			} else {
				upperBounds = upperBounds;									
			}

			if(isTooFar != 0){
				lowerBounds = estimatedAngleDegrees;
			}
		}
	}
	bool isValidAngle = false;									//Control
	if( abs(lowerArmEstematedLength-lengthLowerArm) < 0.1 ) {
		isValidAngle = true;									//Control = True
	} else {
		isValidAngle = false;									//Control = False
	}

	/*	
	Angle II:	
	*/
	double newEstimatedAngleRadians = newEstimatedAngleDegrees * M_PI / 180;	//Angle (in rads)

	basisPositionBallJointRadians = gammaB * M_PI / 180;						//gamma B i (in rads)

	double xPei = x - radiusEffector * cos(desiredRxRadians) * sin(basisPositionBallJointRadians) * sin(desiredRzRadians) 					//X Pei
		- toolCenterPoint * (cos(desiredRzRadians) * sin(desiredRyRadians) + cos(desiredRyRadians) * sin(desiredRxRadians) 
		* sin(desiredRzRadians)) + radiusEffector * cos(basisPositionBallJointRadians) * (cos(desiredRyRadians) * cos(desiredRzRadians)
		- sin(desiredRxRadians) * sin(desiredRyRadians) * sin(desiredRzRadians));
	double yPei = y + radiusEffector * cos(desiredRxRadians) * cos(desiredRzRadians) * sin(basisPositionBallJointRadians) 					//Y Pei
		+ radiusEffector * cos(basisPositionBallJointRadians) * cos(desiredRzRadians) * sin(desiredRxRadians) * sin(desiredRyRadians) 
		- toolCenterPoint * sin(desiredRyRadians) * sin(desiredRzRadians) + cos(desiredRyRadians) * (toolCenterPoint 
		* cos(desiredRzRadians) * sin(desiredRxRadians) + radiusEffector * cos(basisPositionBallJointRadians) * sin(desiredRzRadians));
	double zPei = z + radiusEffector * sin(basisPositionBallJointRadians) * sin(desiredRxRadians) - cos(desiredRxRadians) 					//Z Pei
		* (toolCenterPoint * cos(desiredRyRadians) + radiusEffector * cos(basisPositionBallJointRadians) * sin(desiredRyRadians));	

	double pX = upperArmX - (cos(basisPositionBallJointRadians) * ((cos(newEstimatedAngleRadians) * lengthUpperArm) + radiusBasis));		// P x
	double pY = upperArmY - (sin(basisPositionBallJointRadians) * ((cos(newEstimatedAngleRadians) * lengthUpperArm) + radiusBasis));		// P y
	double pZ = upperArmZ - (-sin(newEstimatedAngleRadians) * lengthUpperArm);																// P z

	double qX = effectorX - xPei;		// Q x
	double qY = effectorY - yPei;		// Q y
	double qZ = effectorZ - zPei;		// Q z

	double pDegrees = acos((pX * qX + pY * qY + pZ * qZ) / (sqrt(pX*pX + pY*pY + pZ*pZ) * sqrt(qX*qX + qY*qY + qZ*qZ))) * 180 / M_PI;		// p (in degrees)
	/*	END Angle II	*/

	/*
	Angle I:
	*/

	double inverseBasisPositionBallJointRadians = (360 * M_PI / 180) - basisPositionBallJointRadians;	// 360 - gamma pi

	lowerArmX = upperArmX - effectorX;	// X L
	lowerArmY = upperArmY - effectorY;	// Y L
	lowerArmZ = upperArmZ - effectorZ;	// Z L

	double temp1 = cos(inverseBasisPositionBallJointRadians) * lowerArmX - sin(inverseBasisPositionBallJointRadians) * lowerArmY;			//tussenstap
	double temp2 = sin(inverseBasisPositionBallJointRadians) * lowerArmX + cos(inverseBasisPositionBallJointRadians) * lowerArmY;			//tussenstap

	double originYL = 0;						// Y 'L

	double lMultipliedByOriginL = temp1 * temp1 + temp2 * originYL + lowerArmZ * lowerArmZ;				// L * 'L
	double lengthLMultipiedByOriginLengthL = sqrt(temp1*temp1 + temp2*temp2 + lowerArmZ*lowerArmZ) 		// |L| * |'L|
		* sqrt(temp1*temp1 + originYL*originYL + lowerArmZ*lowerArmZ);

	double temp3 = lMultipliedByOriginL / lengthLMultipiedByOriginLengthL;

	double p = -1;						// p
	if( temp3 == 1 ){
		p = 0;
	} else { 
		p = acos(lMultipliedByOriginL/lengthLMultipiedByOriginLengthL);
	}

	/*	END Angle I	*/

	bool isValidAngleII = false;					
	if( pDegrees <= ballJointRestriction){
		isValidAngleII = true;			// Angle II isValid = True
	} else {
		isValidAngleII = false;			// Angle II isValid = False
	}

	bool isValidAngleI = false;
	if((p * 180 / M_PI) <= ballJointRestriction){
		isValidAngleI = true;			// Angle I isValid = True
	} else {
		isValidAngleI = false;			// Angle I isValid = False
	}

	bool isTotalValid = false;					
	if(isValidAngle && isValidAngleII && isValidAngleI){
		isTotalValid = true;			// total isValide = False
	} else {
		isTotalValid = false;			// total isValide = False
	}

	double validAngleDegrees = 0;
	if( isTotalValid == true ){
		validAngleDegrees = newEstimatedAngleDegrees;	//Motor succes!
	} else {
		validAngleDegrees = newEstimatedAngleDegrees;
		//validAngleDegrees = -360;						//ERROR!
	}

	return validAngleDegrees;
}

double* MoveCalculations::getOutput(){
	return angles;
}
