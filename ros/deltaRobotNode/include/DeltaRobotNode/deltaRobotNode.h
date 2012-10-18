#ifndef DElTAROBOTNODE_H
#define DELTAROBOTNODE_H

#include "ros/ros.h"
#include "deltaRobotNode/MoveToPoint.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/MoveToRelativePoint.h"
#include "deltaRobotNode/MoveRelativePath.h"
#include "deltaRobotNode/Motion.h"
#include "deltaRobotNode/Calibrate.h"
#include "deltaRobotNode/Calibration.h"

#include <DataTypes/Point3D.h>
#include <DeltaRobot/DeltaRobot.h>
#include <Motor/StepperMotor.h>
#include <DeltaRobotNode/Services.h>
#include <rosMast/StateMachine.h>

namespace deltaRobotNodeNamespace
{
	class DeltaRobotNode : rosMast::StateMachine 
	{
		public:
			DeltaRobotNode(int equipletID, int moduleID) 
				: rosMast::StateMachine(equipletID, moduleID) {
				// FUU
			}
			int transitionSetup();
			int transitionShutdown();
			int transitionStart();
			int transitionStop();
			
			DeltaRobot::DeltaRobot * deltaRobot;
			
			bool calibrate(deltaRobotNode::Calibrate::Request &req, 
				deltaRobotNode::Calibrate::Response &res);
			bool moveToPoint(deltaRobotNode::MoveToPoint::Request &req,
				deltaRobotNode::MoveToPoint::Response &res);
			bool movePath(deltaRobotNode::MovePath::Request &req,
				deltaRobotNode::MovePath::Response &res);
			bool moveToRelativePoint(deltaRobotNode::MoveToRelativePoint::Request &req,
				deltaRobotNode::MoveToRelativePoint::Response &res);
			bool moveRelativePath(deltaRobotNode::MoveRelativePath::Request &req,
				deltaRobotNode::MoveRelativePath::Response &res);	
	};
}

#endif