/**
 * @file deltaRobotNode.h
 * @brief Names for the DeltaRobot services.
 * @date Created: 2012-10-17
 *
 * @author Arjan Groenewegen
 *
 * @section LICENSE
 * License: newBSD
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

#ifndef DELTAROBOTNODE_H
#define DELTAROBOTNODE_H

#include "ros/ros.h"
#include "deltaRobotNode/MoveToPoint.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/MoveToRelativePoint.h"
#include "deltaRobotNode/MoveRelativePath.h"
#include "deltaRobotNode/Motion.h"
#include "deltaRobotNode/Calibrate.h"


#include "rexosStdSrvs/Module.h"

#include <DataTypes/Point3D.h>
#include <DeltaRobot/DeltaRobot.h>
#include <Motor/StepperMotor.h>
#include <DeltaRobotNode/Services.h>
#include <DeltaRobotNode/Point.h>
#include <rosMast/StateMachine.h>
#include <libjson/libjson.h>


namespace deltaRobotNodeNamespace {
	/**
	 * the DeltaRobotNode which is a StateMachine
	 **/
	class DeltaRobotNode : public rosMast::StateMachine {
		public:			
			DeltaRobotNode(int equipletID, int moduleID);
			virtual ~DeltaRobotNode();
			
			int transitionSetup();
			int transitionShutdown();
			int transitionStart();
			int transitionStop();
						
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

			bool calibrateNew(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res);
			bool moveToPointNew(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res);
			bool movePathNew(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res);
			bool moveToRelativePointNew(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res);
			bool moveRelativePathNew(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res);

		private:
			/**
			 * @var DeltaRobot::DeltaRobot * deltaRobot
			 * the deltaRobot
			 **/
			DeltaRobot::DeltaRobot * deltaRobot;
			/**
			 * @var Motor::StepperMotor* motors[3]
			 * the collection of motors
			 **/
			Motor::StepperMotor* motors[3];
			/**
			 *
			 * @var ModbusController::ModbusController* modbus
			 * the modbuscontroller
			 **/
			ModbusController::ModbusController* modbus;
			/**
			 * Motor::MotorManager* motorManager
			 * The motor manager
			 **/
			Motor::MotorManager* motorManager;
			/**
			 * @var ros::ServiceServer moveToPointService;	
			 * Service for receiving move to point commands
			 **/
			ros::ServiceServer moveToPointService;
			/**
			 * @var ros::ServiceServer movePathService;	
			 * Service for receiving movePath commands
			 **/			
			ros::ServiceServer movePathService;
			/**
			 * @var ros::ServiceServer moveToRelativePointService;	
			 * Service for receiving move to relative point commands
			 **/
			ros::ServiceServer moveToRelativePointService;
			/**
			 * @var ros::ServiceServer moveRelativePathService;	
			 * Service for receiving move relative path commands
			 **/			
			ros::ServiceServer moveRelativePathService;
			/**
			 * @var ros::ServiceServer calibrateService;	
			 * Service for receiving calibrate commands
			 **/
			ros::ServiceServer calibrateService;

		

	};
}

#endif