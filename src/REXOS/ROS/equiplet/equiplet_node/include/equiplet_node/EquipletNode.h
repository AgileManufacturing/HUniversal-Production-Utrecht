/**
 * @file EquipletNode.h
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
 *
 * @author Dennis Koole
 * @author Gerben Boot & Joris Vergeer
 *
 * @section LICENSE
 * License: newBSD
 *
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

#pragma once

#include "ros/ros.h"
#include "environment_cache/getData.h"
#include "rexos_logger/rexos_logger.h"

#include <string>
#include <vector>

#include <rexos_node_spawner/NodeSpawner.h>
#include <rexos_datatypes/HardwareStep.h>
#include <rexos_utilities/Utilities.h>

#include <equiplet_node/state_machine/EquipletStateMachine.h>
#include <equiplet_node/ModuleRegistry.h>
#include <equiplet_node/scada/EquipletScada.h>
#include <equiplet_node/NodeHalInterface.h>
#include <equiplet_node/BlackBoardHalInterface.h>
#include <equiplet_node/HalInterfaceListener.h>

#include <jsoncpp/json/value.h>

#include <actionlib/client/simple_action_client.h>

namespace equiplet_node {
#define ROS_INTERFACE_IMPLEMENTATION_PATH "rosInterface/usedImplementation/"
#define BLACKBOARD_IMPLEMENTATION "blackboard"
#define BRIDGE_IMPLEMENTATION "rosBridge"
#define JAVA_NODE_IMPLEMENTATION "javaNode"

/**
 * The equipletNode, will manage all modules and keep track of their states
 **/
class EquipletNode : public EquipletStateMachine, protected HalInterfaceListener
{
public:
	EquipletNode(std::string equipletName, bool isSimulated, bool isShadow, std::string blackboardIp, int scadaPort);
	~EquipletNode();
	
	std::string getEquipletName();
	ros::NodeHandle& getNodeHandle();
	
private:
	void handleHardwareStep(rexos_datatypes::HardwareStep& step, mongo::OID targetObjectId);
	void onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, rexos_datatypes::HardwareStep hardwareStep);
	
	virtual void onStateChanged(rexos_statemachine::State state);
	virtual void onModeChanged(rexos_statemachine::Mode mode);
	
	Json::Value callLookupHandler(Json::Value originPlacementParameters);
	
	bool setTransitionDone(rexos_statemachine::State transitionState);
	
	void handleEquipletCommand(Json::Value n);
	
	std::string equipletName;
	bool isSimulated;
	bool isShadow;
	
	ros::NodeHandle nh;
	equiplet_node::scada::EquipletScada scada;
	HalInterface* halInterface;
	ros::ServiceClient spawnPartClient;
	ros::ServiceClient removePartClient;
	
	ros::Subscriber accelerationViolationSubscriber;
	ros::Subscriber collisionViolationSubscriber;
	ros::Subscriber jointViolationSubscriber;
	
	virtual void onHardwareStep(rexos_datatypes::HardwareStep hardwareStep);
	virtual void onEquipletCommand(rexos_datatypes::EquipletCommand equipletCommand);
	void onModuleModeChanged(rexos_module::ModuleProxy* moduleProxy, 
		rexos_statemachine::Mode newMode, rexos_statemachine::Mode prevMode);
	void onModuleStateChanged(rexos_module::ModuleProxy* moduleProxy, 
		rexos_statemachine::State newState, rexos_statemachine::State prevState);
	void onAccelerationViolation(std_msgs::String message);
	void onCollisionViolation(std_msgs::String message);
	void onJointViolation(std_msgs::String message);
};

}
