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
#include "lookup_handler/LookupServer.h"

#include <string>
#include <vector>

#include <rexos_blackboard_cpp_client/BlackboardCppClient.h>
#include <rexos_blackboard_cpp_client/BlackboardSubscriber.h>
#include <rexos_blackboard_cpp_client/FieldUpdateSubscription.h>
#include <rexos_datatypes/EquipletStep.h>
#include <rexos_utilities/Utilities.h>

#include <equiplet_node/state_machine/EquipletStateMachine.h>
#include <equiplet_node/ModuleRegistry.h>
#include <equiplet_node/ModuleProxy.h>
#include <equiplet_node/scada/EquipletScada.h>

#pragma GCC system_header
#include <libjson/libjson.h>

#include <actionlib/client/simple_action_client.h>

namespace equiplet_node {

/**
 * The equipletNode, will manage all modules and keep track of their states
 **/
class EquipletNode : public EquipletStateMachine,public Blackboard::BlackboardSubscriber
{
public:
	static std::string nameFromId(int id){
		return std::string("equiplet_") + std::to_string(id);
	}

	EquipletNode(int id, std::string blackboardIp);

	virtual ~EquipletNode();

	void blackboardReadCallback(std::string json);

	std::string getName();

	ros::NodeHandle& getNodeHandle();

	void onInstructionStepCompleted(ModuleProxy* moduleProxy, std::string id, bool completed);

private:
	virtual void onStateChanged();
	virtual void onModeChanged();

	void callLookupHandler(std::string lookupType, std::string lookupID, std::map<std::string, std::string> payloadMap);

	void onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry);

	environment_communication_msgs::Map createMapMessage(std::map<std::string, std::string> &Map);
	bool setTransitionDone(rexos_statemachine::State transitionState);

	void updateEquipletStateOnBlackboard();

	/**
	 * testdata
	 **/
	int amountOfIncomingMongoDBCalls = 0;

	/**
	 * @var int equipletId
	 * The id of the equiplet
	 **/
	int equipletId;

	/**
	 * @var BlackboardCppClient  *blackboardClient
	 * Client to read from blackboard
	 **/

	Blackboard::BlackboardCppClient *equipletStepBlackboardClient;
	Blackboard::FieldUpdateSubscription* equipletStepSubscription;

	Blackboard::BlackboardCppClient *equipletCommandBlackboardClient;
	Blackboard::BlackboardSubscription* equipletCommandSubscription; 
	Blackboard::BlackboardSubscription* equipletCommandSubscriptionSet; 

	Blackboard::BlackboardCppClient *equipletStateBlackboardClient;
	std::vector<Blackboard::BlackboardSubscription *> subscriptions; 

	equiplet_node::scada::EquipletScada scada;

	ros::NodeHandle nh;
};

}
