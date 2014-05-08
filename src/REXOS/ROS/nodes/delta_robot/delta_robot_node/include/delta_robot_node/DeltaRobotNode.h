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
#include "rexos_std_srvs/Module.h"
#include "delta_robot_node/Point.h"

#include <rexos_datatypes/Point3D.h>
#include <rexos_delta_robot/DeltaRobot.h>
#include <rexos_motor/StepperMotor.h>
#include <rexos_motor/StepperMotorProperties.h>
#include <delta_robot_node/Services.h>
#include <delta_robot_node/Point.h>
#include <rexos_statemachine/ModuleStateMachine.h>
#include <rexos_statemachine/Transitions.h>
#include <rexos_coordinates/Module.h>
#include "equiplet_node/RegisterModule.h"

#include <actionlib/server/simple_action_server.h>
#include <rexos_statemachine/SetInstructionAction.h>

// GCC system header to suppress libjson warnings
#pragma GCC system_header
#include <libjson/libjson.h>
// ---------------------------------------------


namespace deltaRobotNodeNamespace{

	typedef actionlib::SimpleActionServer<rexos_statemachine::SetInstructionAction> SetInstructionActionServer;

	/**
	 * the DeltaRobotNode which is a ModuleStateMachine
	 **/
	class DeltaRobotNode : public rexos_statemachine::ModuleStateMachine, 
			public rexos_knowledge_database::Module, 
			public rexos_coordinates::Module{
	protected:
		rexos_motor::StepperMotorProperties* stepperMotorProperties;
		rexos_datatypes::DeltaRobotMeasures* deltaRobotMeasures;
	
		std::string modbusIp;
		int modbusPort;
		
		int calibrationBigStepFactor;
		
		
	public:
		DeltaRobotNode(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier);
		virtual ~DeltaRobotNode();
		
		virtual void transitionInitialize(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionDeinitialize(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionSetup(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionShutdown(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionStart(rexos_statemachine::TransitionActionServer* as);
		virtual void transitionStop(rexos_statemachine::TransitionActionServer* as);
			
		// Main functions to be called from the services
		bool calibrate();
		bool moveToPoint(double x, double y, double z, double maxAcceleration);
		bool moveToRelativePoint(double x, double y, double z, double maxAcceleration);

		Point parsePoint(const JSONNode & n, std::string * valuesSet);
		std::string parseNodeValue(const std::string nodeName, const JSONNode & n);
		Point parseLookup(const JSONNode & n);
		Point *parsePointArray(std::string json, int & size);

		void onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal);

	private:
		float lastX;
		float lastY;
		float lastZ;
		std::string moduleNodeName;
		/**
		 * @var ros::NodeHandle node
		 * The nodeHandle used by ros services and topics
		 **/
		ros::NodeHandle nodeHandle;
		SetInstructionActionServer setInstructionActionServer;
		/**
		 * @var DeltaRobot::DeltaRobot * deltaRobot
		 * the deltaRobot
		 **/
		rexos_delta_robot::DeltaRobot * deltaRobot;
	};
}
#endif
