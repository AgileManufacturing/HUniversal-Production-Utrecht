/**
 * @file GripperNode.h
 * @brief GripperNode for controlling the gripper and accepting services.
 * @date Created: 2012-11-05
 *
 * @author Koen Braham
 * @author Dick vd Steen
 * @author Alexander Streng
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

#ifndef GRIPPERNODE_H_
#define GRIPPERNODE_H_

#include "rexos_std_srvs/Module.h"
#include "ros/ros.h"
#include "gripper_node/Grip.h"
#include "gripper_node/Release.h"
#include "Services.h"
#include "iostream"
#include <rexos_gripper/Gripper.h>
#include <rexos_statemachine/ModuleStateMachine.h>
#include <rexos_statemachine/Transitions.h>
#include <rexos_knowledge_database/Module.h>
#include "equiplet_node/RegisterModule.h"
#include "rexos_logger/rexos_logger.h"

#include <actionlib/server/simple_action_server.h>
#include <rexos_statemachine/SetInstructionAction.h>


// GCC system header to suppress libjson warnings
#pragma GCC system_header
#include <libjson/libjson.h>
/**
 * Gripper node that provides services to control the gripper valve
 **/
typedef actionlib::SimpleActionServer<rexos_statemachine::SetInstructionAction> SetInstructionActionServer;

class GripperNode : public rexos_statemachine::ModuleStateMachine, 
		public rexos_knowledge_database::Module  {
public:

	GripperNode(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier);
	virtual ~GripperNode();

	virtual bool transitionInitialize();
	virtual bool transitionDeinitialize();
	virtual bool transitionSetup();
	virtual bool transitionShutdown();
	virtual bool transitionStart();
	virtual bool transitionStop();

	void error();
	static void wrapperForGripperError(void* gripperNodeObject);
	
	bool grip(gripper_node::Grip::Request &req, gripper_node::Grip::Response &res);
	bool release(gripper_node::Release::Request &req, gripper_node::Release::Response &res);

	void onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal);
	std::string parseNodeValue(const std::string nodeName, const JSONNode & n);

private:
	/**
	 * @var modbus_t* modbusContext
	 * Connection to the IO modbus
	 **/
	modbus_t* modbusContext;


	SetInstructionActionServer setInstructionActionServer;

	/**
	 * @var InputOutput::OutputDevices::Gripper* gripper
	 * The gripper device
	 **/
	rexos_gripper::Gripper* gripper;

	/**
	 * @var ModbusController::ModbusController* modbus
	 * Connection to the IO modbus controller
	 **/
	rexos_modbus::ModbusController* modbus;

	/**
	 * @var InputOutput::InputOutputController* controller;
	 * Input output controller that contains the logic to switch devices
	 */
	rexos_gripper::InputOutputController* controller;

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

#endif /* GRIPPERNODE_H_ */
