/**
 * @file MotorJoint.h
 * @brief this object simulates a real life motor (stepper, dc, servo, etc) in gazebo. It does this by taking a joint of a
 * given gazebo robot and manipulating its rotations limits to  make it rotate and hold the current angle.
 * @date Created: 2013-10-30
 * @date Revisioned: 2013-01-21
 *
 * @author Garik Hakopian
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
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
 **/

#ifndef SIMULATEDMOTOR_H_
#define SIMULATEDMOTOR_H_

#include "rexos_gazebo/SDFController.h"
#include "gazebo_msgs/LinkState.h"

#include <vector>

class SimulatedMotor{
	
public:
	SimulatedMotor();
	
	/**
	* initialises the motor with the parameters required for it to function
	* @param const char * jointName - the joint to manipulate as a motor
	**/
	void init(const char * jointName);
	
	/**
	* rotates the motor (the joint this instance is controlling) to a given angle in degrees
	* @param double degrees
	**/
	void goToAngleDegrees(double degrees);
	
	/**
	* rotates the motor (the joint this instance is controlling) to a given angle in radians
	* @param double radians
	**/
	void goToAngleRadians(double radians);
	
	/**
	* updates the position of the motor. This method checks if the current position of the motor 
	* is up to date with the simulation and updates it accordingly
	**/
	void update();
	
private:
	SDFController sdfController;
	const char * name;
	double currentAngle;
	double destinationAngle;
	
	double rotationDestination;
	
	
};

#endif
