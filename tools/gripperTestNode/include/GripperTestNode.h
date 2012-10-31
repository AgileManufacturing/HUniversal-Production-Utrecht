#ifndef GRIPPERTESTNODE_H
#define GRIPPERTESTNODE_H

#include <rosMast/StateMachine.h>
#include "ros/ros.h"
#include "gripperTestNode/Grip.h"
#include "gripperTestNode/Release.h"
#include "Services.h"
#include "iostream"
#include "Gripper.h"

class GripperTestNode: public rosMast::StateMachine
{
	public:
		GripperTestNode(int equipletID, int moduleID);
		~GripperTestNode();
		int transitionSetup();
		int transitionShutdown();
		int transitionStart();
		int transitionStop();
		void error();
		static void WrapperForGripperError(void* gripperNodeObject);
		bool grip(gripperTestNode::Grip::Request &req, gripperTestNode::Grip::Response &res);	
		bool release(gripperTestNode::Release::Request &req, gripperTestNode::Release::Response &res);
	private:
		/**
		 * @var Gripper * gripper
		 * The gripper device
		 **/
		Gripper * gripper;
		/**
		 * @var ros::ServiceServer gripService
		 * The service for enabling the gripper
		 **/
		ros::ServiceServer gripService;
		/**
		 * @var ros::ServiceServer releaseService
		 * The service for releasing the gripper
		 **/
		ros::ServiceServer releaseService;
};

#endif