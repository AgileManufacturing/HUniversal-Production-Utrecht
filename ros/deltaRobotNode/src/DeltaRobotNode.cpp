/*
 * DeltaRobotNode.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: Dick van der Steen & Dennis Koole
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
				huniplacer::point3(currentMotion.x, currentMotion.y, currentMotion.z),
				huniplacer::point3(nextMotion.x, nextMotion.y, nextMotion.z)))
			{
				res.succeeded = false;
				return true;
			}
		}
		for(n = 0; n < req.motion.size(); n++)
		{	
			currentMotion = req.motion[n];
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", currentMotion.x, currentMotion.y,currentMotion.z, currentMotion.speed);
			deltarobot->moveto(huniplacer::point3(currentMotion.x, currentMotion.y, currentMotion.z),currentMotion.speed);
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


