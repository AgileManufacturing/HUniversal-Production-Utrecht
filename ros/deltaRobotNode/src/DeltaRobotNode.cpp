/*
 * DeltaRobotNode.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: Dick van der Steen & Dennis Koole

// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************
*/

#include "ros/ros.h"
#include <huniplacer/huniplacer.h> 
#include "deltaRobotNode/MoveToPoint.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/MoveToRelativePoint.h"
#include "deltaRobotNode/MoveRelativePath.h"
#include "deltaRobotNode/Motion.h"

#define NODE_NAME "DeltaRobotNode"

 static huniplacer::deltarobot * deltarobot;


/*
 *
 */
bool moveToPoint(deltaRobotNode::MoveToPoint::Request &req,
	deltaRobotNode::MoveToPoint::Response &res) {

	return true;
}

/*
 *
 */
bool movePath(deltaRobotNode::MovePath::Request &req,
	deltaRobotNode::MovePath::Response &res) {
	ROS_INFO("movePath called");
	res.succeeded = true;

	deltaRobotNode::Motion currentMotion;
	deltaRobotNode::Motion nextMotion;
	try
	{
		unsigned int n;
		for(n = 0; n < req.motion.size() -1; n++)
		{
			currentMotion = req.motion[n];			
			nextMotion = req.motion[n+1];
			if(!deltarobot->check_path(
				huniplacer::Point3D(currentMotion.x, currentMotion.y, currentMotion.z),
				huniplacer::Point3D(nextMotion.x, nextMotion.y, nextMotion.z)))
			{
				res.succeeded = false;
				return true;
			}
		}
		for(n = 0; n < req.motion.size(); n++)
		{	
			currentMotion = req.motion[n];
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", currentMotion.x, currentMotion.y,currentMotion.z, currentMotion.speed);
			deltarobot->moveto(huniplacer::Point3D(currentMotion.x, currentMotion.y, currentMotion.z),currentMotion.speed);
		}
		//deltaRobotNode::Motion msg;
		//msg = req.motion; 		
		//pubDeltaPos->publish(msg);
		deltarobot->wait_for_idle();
	}
	catch(std::runtime_error& ex)
	{
		//deltarobotnode::error msg;
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		//msg.errorMsg = ss.str();
		//msg.errorType = 2;
		//pub->publish(msg);
		res.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}
	return true;
}

/*
 *
 */
bool moveToRelativePoint(deltaRobotNode::MoveToRelativePoint::Request &req,
	deltaRobotNode::MoveToRelativePoint::Response &res) {

	return true;
}

/*
 *
 */
bool moveRelativePath(deltaRobotNode::MoveRelativePath::Request &req,
	deltaRobotNode::MoveRelativePath::Response &res) {

	return true;
}

/*
 *
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	ros::ServiceServer moveToPointService =
		nodeHandle.advertiseService("moveToPoint", moveToPoint);

	ros::ServiceServer movePathService =
		nodeHandle.advertiseService("movePath", movePath);

	ros::ServiceServer moveToRelativePointService =
		nodeHandle.advertiseService("moveToRelativePoint", moveToRelativePoint);

	ros::ServiceServer moveRelativePathService =
		nodeHandle.advertiseService("moveRelativePath", moveRelativePath);

	ROS_INFO("DeltaRobotNode ready.");

	ros::spin();
	return 0;
}


