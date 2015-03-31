/**
 * @file GazeboSDF.h
 * @brief The SDFController handles services provided by gazebo to provide an interface for controlling a gazebo simulated robot
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

#ifndef SDFCONTROLLER_H_
#define SDFCONTROLLER_H_

#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/SetJointProperties.h"
#include "gazebo_msgs/ODEJointProperties.h"
#include "gazebo_msgs/JointRequest.h"
#include "gazebo_msgs/GetLinkState.h"
#include "rexos_logger/rexos_logger.h"

#include <vector>


/**
* Struct containing joint properties used by the SDFController
* @param int type - the type of the joint
* @param double damping;
* @param double position;
* @param double rate;
**/
struct JointProperties {
	int type;
	double damping;
    double position;
    double rate;
	JointProperties(int tp, double damp, double pos, double rt){
		type = tp;
		damping = damp;
		position = pos;
		rate = rt;
	}
} ;

class SDFController{
	
public:
	SDFController();

	/**
	* Returns a JointProperties object containing the properties of the joint specified by jointName
	* @param const char * linkName
	**/
	JointProperties getJointProperties(const char * jointName);
	
	/**
	* Returns a gazebo_msgs::LinkState object containing the properties of the link specified by linkName
	* @param const char * linkName
	* 
	* LinkState contains: link_name(), pose(), twist(), reference_frame()
	* where pose contains position.x or y,z
	**/
	gazebo_msgs::LinkState getLinkState(const char * linkName);

	/**
	* sets the state of the link specified by linkName
	* @param const char * linkName
	* @param gazebo_msgs::LinkState state
	**/
	void setLinkState(const char * linkName, gazebo_msgs::LinkState state);
	
	/**
	* sets the properties of the joint specified by jointName
	* @param const char * jointName
	* @param gazebo_msgs::ODEJointProperties jointConfig
	**/
	void setJointProperties(const char * jointName, gazebo_msgs::ODEJointProperties jointConfig);
	
	/**
	* applies force to the joint specified by jointName
	* @param const char * jointName
	* @param double force
	* @param double durationTime in ms
	**/
	void applyForceToJoint(const char * jointName, double force, double durationTime);
	
	/**
	* Rotates the joint specified by jointName by a given speed. Negative speed will rotate to the counter direction
	* @param const char * jointName
	* @param double speed
	**/
	void rotateJoint(const char * jointName, double speed);

private:
	

};


#endif
