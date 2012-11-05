/*
 * GripperNode.h
 *
 *  Created on: Nov 5, 2012
 *      Author: kbraham
 */

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
	ModbusController::ModbusController* modbus;

	/**
	 * @var Gripper gripper
	 * The gripper device
	 **/
	InputOutput::OutputDevices::Gripper* gripper;
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
