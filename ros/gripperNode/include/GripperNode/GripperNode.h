/**
 * @file GripperNode.h
 * @brief GripperNode for controlling the gripper and accepting services.
 * @date Created: 2012-11-05
 *
 * @author Koen Braham
 * @author Dick vd Steen
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

#include <rosMast/StateMachine.h>
#include "ros/ros.h"
#include "gripperNode/Grip.h"
#include "gripperNode/Release.h"
#include "Services.h"
#include "iostream"
#include <InputOutput/OutputDevices/Gripper.h>

class GripperNode: public rosMast::StateMachine {
public:
	GripperNode(int equipletID, int moduleID);
	virtual ~GripperNode( );
	int transitionSetup( );
	int transitionShutdown( );
	int transitionStart( );
	int transitionStop( );
	void error( );
	static void wrapperForGripperError(void* gripperNodeObject);
	bool grip(gripperNode::Grip::Request &req, gripperNode::Grip::Response &res);
	bool release(gripperNode::Release::Request &req, gripperNode::Release::Response &res);

private:
	modbus_t* modbusContext;
	InputOutput::OutputDevices::Gripper* gripper;
	ModbusController::ModbusController* modbus;
	InputOutput::InputOutputController* controller;

	/**
	 * @var Gripper gripper
	 * The gripper device
	 **/

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
