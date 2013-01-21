/**
 * @file Gripper.cpp
 * @brief Gripper test node
 * @date Created: 2012-10-17
 * 
 * @author Dick van der Steen
 * @author Koen Braham
 * 
 * @section LICENSE
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

#include "GripperTestNode.h"
#include <Utilities/Utilities.h>

#define NODE_NAME "GripperTestNode"

/**
 * Constructor 
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the gripper
 **/
GripperTestNode::GripperTestNode(int equipletID, int moduleID): rosMast::StateMachine(equipletID, moduleID){
	gripper = new Gripper(this, WrapperForGripperError);
	ros::NodeHandle nodeHandle;
	// Advertise the services
	gripService = nodeHandle.advertiseService(GripperTestNodeServices::GRIP, &GripperTestNode::grip, this);
	releaseService = nodeHandle.advertiseService(GripperTestNodeServices::RELEASE, &GripperTestNode::release, this);
}

GripperTestNode::~GripperTestNode(){
	delete gripper;
}

/** 
 * A wrapper for the gripper error handler so that we can use a member function
 * @param gripperNodeObject pointer to the gripperTestNode object
 **/
void GripperTestNode::WrapperForGripperError(void* gripperNodeObject){
	GripperTestNode* myself = (GripperTestNode*) gripperNodeObject;
	myself->error();
}

/** 
 * Sends error message to equipletNode with an errorcode
 **/
void GripperTestNode::error(){
	sendErrorMessage(-1);
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionSetup(){
	ROS_INFO("Setup transition called");
	setState(rosMast::setup); 
	return 0; 
}

/**
 * Transition from Standby to Safe state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionShutdown(){
	ROS_INFO("Shutdown transition called");
	setState(rosMast::shutdown);
	return 0;
}

/**
 * Transition from Standby to Normal state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionStart(){
	ROS_INFO("Start transition called");
	setState(rosMast::start);
	return 0;
}

/**
 * Transition from Normal to Standby state
 * @return 0 if everything went OK else error
 **/
int GripperTestNode::transitionStop(){
	ROS_INFO("Stop transition called");
	gripper->release();
	setState(rosMast::stop);
	return 0;
}

/**
 * Set gripper on
 *
 * @param req The request for this service as defined in Grip.srv
 * @param res The response for this service as defined in Grip.srv
 *
 * @return true if gripper is put on else return false.
 **/
bool GripperTestNode::grip(gripperTestNode::Grip::Request &req, gripperTestNode::Grip::Response &res){
	res.succeeded = false;
	if(getState() == rosMast::normal){
		gripper->grab();
		res.succeeded = true;
	} else{
		res.succeeded = false;
	}
	return true;
}

/**
 * Set gripper off
 *
 * @param req The request for this service as defined in Grip.srv 
 * @param res The response for this service as defined in Grip.srv
 *
* @return true if gripper is put off else return false.
 **/
bool GripperTestNode::release(gripperTestNode::Release::Request &req, gripperTestNode::Release::Response &res){
	res.succeeded = false;
	if(getState() == rosMast::normal){
		gripper->release();
		res.succeeded = true;
		ROS_INFO("Gripper released");
	} else{
		res.succeeded = false;
		ROS_INFO("Gripper not released, state was not normal");
	}
	return true;
}

int main(int argc, char** argv){
	ros::init(argc, argv, NODE_NAME);
	int equipletID = 0;
	int moduleID = 0;
	if(argc < 3 || !(Utilities::stringToInt(equipletID, argv[1]) == 0 && Utilities::stringToInt(moduleID, argv[2]) == 0)){
		std::cerr << "Cannot read equiplet id and/or moduleId from commandline please use correct values." << std::endl;
		return -1;
	}

	GripperTestNode gripperTestNode(equipletID, moduleID);

	ros::spin();
	return 0;
}
