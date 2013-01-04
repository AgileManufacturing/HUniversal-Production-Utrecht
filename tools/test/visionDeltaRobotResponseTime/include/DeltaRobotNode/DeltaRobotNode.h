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
#include "visionDeltaRobotResponseTime/MoveToPoint.h"
#include "visionDeltaRobotResponseTime/MovePath.h"
#include "visionDeltaRobotResponseTime/MoveToRelativePoint.h"
#include "visionDeltaRobotResponseTime/MoveRelativePath.h"
#include "visionDeltaRobotResponseTime/Motion.h"
#include "visionDeltaRobotResponseTime/Calibrate.h"

#include <DataTypes/Point3D.h>
#include <DeltaRobot/DeltaRobot.h>
#include <Motor/StepperMotor.h>
#include <DeltaRobotNode/Services.h>
#include <rosMast/StateMachine.h>

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
						
			bool calibrate(visionDeltaRobotResponseTime::Calibrate::Request &req,
					visionDeltaRobotResponseTime::Calibrate::Response &res);
			bool moveToPoint(visionDeltaRobotResponseTime::MoveToPoint::Request &req,
					visionDeltaRobotResponseTime::MoveToPoint::Response &res);
			bool movePath(visionDeltaRobotResponseTime::MovePath::Request &req,
					visionDeltaRobotResponseTime::MovePath::Response &res);
			bool moveToRelativePoint(visionDeltaRobotResponseTime::MoveToRelativePoint::Request &req,
					visionDeltaRobotResponseTime::MoveToRelativePoint::Response &res);
			bool moveRelativePath(visionDeltaRobotResponseTime::MoveRelativePath::Request &req,
					visionDeltaRobotResponseTime::MoveRelativePath::Response &res);
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
