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
#include "gripperNode/Grip.h"
#include "gripperNode/Release.h"
#include "Services.h"
#include "iostream"
#include <rexos_statemachine/ModuleStateMachine.h>
#include <InputOutput/OutputDevices/Gripper.h>
#include "equiplet_node/RegisterModule.h"

#include <actionlib/server/simple_action_server.h>
#include <rexos_statemachine/SetInstructionAction.h>

/**
 * Gripper node that provides services to control the gripper valve
 **/
typedef actionlib::SimpleActionServer<rexos_statemachine::SetInstructionAction> SetInstructionActionServer;

class GripperNode : public rexos_statemachine::ModuleStateMachine  {
public:

	GripperNode(int equipletID, int moduleID);
	virtual ~GripperNode();

	virtual void transitionSetup(rexos_statemachine::TransitionActionServer* as);
	virtual void transitionShutdown(rexos_statemachine::TransitionActionServer* as);
	virtual void transitionStart(rexos_statemachine::TransitionActionServer* as);
	virtual void transitionStop(rexos_statemachine::TransitionActionServer* as);

	void error();
	static void wrapperForGripperError(void* gripperNodeObject);
	
	bool grip(gripperNode::Grip::Request &req, gripperNode::Grip::Response &res);
	bool release(gripperNode::Release::Request &req, gripperNode::Release::Response &res);

	void onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal);

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
	InputOutput::OutputDevices::Gripper* gripper;

	/**
	 * @var ModbusController::ModbusController* modbus
	 * Connection to the IO modbus controller
	 **/
	ModbusController::ModbusController* modbus;

	/**
	 * @var InputOutput::InputOutputController* controller;
	 * Input output controller that contains the logic to switch devices
	 */
	InputOutput::InputOutputController* controller;

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
