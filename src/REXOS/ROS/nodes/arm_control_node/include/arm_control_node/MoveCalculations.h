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

#ifndef MOVECALCULATIONS_H
#define MOVECALCULATIONS_H

class MoveCalculations
{

public:

	/**
	* Default constructor for the SixAxis class
	*
	* Default constructor that uses the variables given in the Excel worksheet "Calculations_Matrices_Design I_07-06-2013_Excel_Brute_Force.xlsx" created by J.H. van Duijn.
	* CAUTION: When using this default constructor make 100% sure that the SixAxis actually has these settings!
	*/
	MoveCalculations();

	/**
	* Constructor for the SixAxis class
	*
	* Constructor that uses given variables as settings for the SixAxis.
	* CAUTION: When using this constructor make 100% sure that the SixAxis actually has the given settings! 
	* @param gammaB1 Position of ball-joint #1 on the basis (in degrees)
	* @param gammaB2 Position of ball-joint #2 on the basis (in degrees)
	* @param gammaB3 Position of ball-joint #3 on the basis (in degrees)
	* @param gammaB4 Position of ball-joint #4 on the basis (in degrees)
	* @param gammaB5 Position of ball-joint #5 on the basis (in degrees)
	* @param gammaB6 Position of ball-joint #6 on the basis (in degrees)
	* @param eB1 Distance between balljoint #1 and arm (in mm)
	* @param eB2 Distance between balljoint #2 and arm (in mm)
	* @param eB3 Distance between balljoint #3 and arm (in mm)
	* @param eB4 Distance between balljoint #4 and arm (in mm)
	* @param eB5 Distance between balljoint #5 and arm (in mm)
	* @param eB6 Distance between balljoint #6 and arm (in mm)
	* @param rb Radius of the Basis (in mm)
	* @param tCP Tool Center Point (in mm)
	* @param minimumAngle Minimum Angle	(in degrees)
	* @param maximumAngle Maximum Angle (in degrees)
	* @param uArmLength Length upper arm (in mm)
	* @param lArmLength Length lower arm (in mm)
	* @param ballJointsRestriction Ball-joints restrictions (in degrees)
	* @param rp Radius of the Effector (in mm)
	*/
	MoveCalculations(double gammaB1, double gammaB2, double gammaB3, double gammaB4, double gammaB5, double gammaB6, double eB1, double eB2, double eB3, double eB4, double eB5, double eB6, int rb, int tCP, int minimumAngle, int maximumAngle, int uArmLength, int lArmLength, int ballJointsRestrictions, int rp);

	/**
	* Getter for the succesfully calculated motor angles
	*
	* @return pointer with 6 doubles, one for each succesfully calculated motor angles
	*/
	double* getOutput();

	/**
	* OLD - Function that will attempt to move the effector to the given point
	*
	* Function that will calculate each motor's angle to get to the given point, if the point isn't out of bounds it will return true.
	* @param x The Center Tool Point's desired location in the X-axis ( in mm )
	* @param y The Center Tool Point's desired location in the Y-axis ( in mm )
	* @param z The Center Tool Point's desired location in the Z-axis ( in mm )
	* @param rX The Center Tool Point's desired pitch ( in degrees )
	* @param rY The Center Tool Point's desired roll (in degrees )
	* @param rZ The Center Tool Point's desired yaw ( in degrees )
	* @return true if all 6 motors can move succesfully
	*/
	bool moveEffectorToPoint(double x, double y, double z, double rX, double rY, double rZ);

	/**
	* Function that will attempt to move the effector to the given point, turning the motors starting from the last angles they were in
	*
	* Function that will calculate each motor's angle to get to the given point, if the point isn't out of bounds it will return true.
	* @param x The Center Tool Point's desired location in the X-axis ( in mm )
	* @param y The Center Tool Point's desired location in the Y-axis ( in mm )
	* @param z The Center Tool Point's desired location in the Z-axis ( in mm )
	* @param rX The Center Tool Point's desired pitch ( in degrees )
	* @param rY The Center Tool Point's desired roll (in degrees )
	* @param rZ The Center Tool Point's desired yaw ( in degrees )
	* @return true if all 6 motors can move succesfully
	*/
	bool moveEffectorToRelativePoint(double x, double y, double z, double rX, double rY, double rZ);


private: 

	double lastAngles[6];
	double angles[6];
	double basisPositionBallJoints[6];
	double distancesBallJointsAndArms[6];

	double lastX;
	double lastY;
	double lastZ;
	double lastRX;
	double lastRY;
	double lastRZ;

	double radiusBasis, toolCenterPoint, minAngle, maxAngle, lengthUpperArm, lengthLowerArm, ballJointRestriction, radiusEffector;
	
	/**
	* Function that calculates the angle required to get to the given location, pitch, roll and yaw of a motor on the given location
	*
	* Function that calculates and returns the angle required to get to the given location, pitch, roll and yaw of a motor on the given location
	* @param x The Center Tool Point's desired location in the X-axis ( in mm )
	* @param y The Center Tool Point's desired location in the Y-axis ( in mm )
	* @param z The Center Tool Point's desired location in the Z-axis ( in mm )
	* @param rX The Center Tool Point's desired pitch ( in degrees )
	* @param rY The Center Tool Point's desired roll (in degrees )
	* @param rZ The Center Tool Point's desired yaw ( in degrees )
	* @param gammaB Position of ball-joint on the basis (in degrees)
	* @param eP Distance between the ball-joint and arm (in mm)
	* @return angle required to get to the given point, -360 if point is out of bounds
	*/
	double calcMotorAngle(double x, double y, double z, double rX, double rY, double rZ, double gammaB, double eP);
};

#endif