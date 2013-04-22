/**
 * @file FollowNode.cpp
 * @brief Listens to the crateEvent topic and guides the deltarobot
 * @date Created: 2012-10-30
 *
 * @author Koen Braham
 * @author Daan Veltman
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

#include <FollowNode/FollowNode.h>

#include <crate_locator_node/Topics.h>
#include <crate_locator_node/Services.h>
#include <delta_robot_node/Services.h>
#include <rexos_datatypes/Crate.h>
#include <rexos_vision/CrateTracker.h>
#include "delta_robot_node/MoveToPoint.h"

FollowNode::FollowNode() :
	nodeHandle(),
	crateID(""), 
	updateCrateIDFlag(false), 
	inputRunning(true), 
	topicRunning(true),
	inputThread(NULL), 
	deltaRobotClient(nodeHandle.serviceClient<delta_robot_node::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)),
	crateLocatorClient(nodeHandle.serviceClient<crate_locator_node::getCrate>(CrateLocatorNodeServices::GET_CRATE)),
	getCrateService(){
	inputThread = new boost::thread(inputThreadMethod, this);

	getCrateService.request.name = "";

	/*
	moveToPointService.request.motion.x = 0;
	moveToPointService.request.motion.y = 0;
	moveToPointService.request.motion.z = -230;
	moveToPointService.request.motion.maxAcceleration = 75;
	*/
}

FollowNode::~FollowNode(){
	inputRunning = false;
	topicRunning = false;
	inputThread->interrupt();
}

/**
 * Function to act on received Crate Event messages
 *
 * @param msg CrateEventMsg pointer.
 **/
void FollowNode::callback(const crate_locator_node::CrateEventMsg::ConstPtr& msg){
	switch(msg->event){
	case rexos_vision::CrateEvent::type_in:
		std::cout << "[DEBUG] New crate " << msg->crate.name << "found!" << std::endl;
		if(updateCrateIDFlag){
			crateID = msg->crate.name;
			getCrateService.request.name = crateID;
			std::cout << "[DEBUG] Tracking new crate " << crateID << std::endl;
			updateCrateIDFlag = false;
		}

		// Move to the crate!
		/*if(crateID.compare(msg->crate.name) == 0){
		 std::cout << "[DEBUG] Moving to new coordinate " << msg->crate.x << "," << msg->crate.y << std::endl;
		 moveToPointService.request.motion.x = msg->crate.x;
		 moveToPointService.request.motion.y = msg->crate.y;
		 deltaRobotClient.call(moveToPointService);
		 }*/
		break;
	case rexos_vision::CrateEvent::type_out:
		std::cout << "[DEBUG] Deleted crate " << msg->crate.name << std::endl;
		//if(crateID.compare(msg->crate.name) == 0){
		//	crateID = "";
		//	std::cout << "[DEBUG] Lost crate " << msg->crate.name << ". Stopped the follow process " << std::endl;
		//}
		break;
	case rexos_vision::CrateEvent::type_moving:
		std::cout << "[DEBUG] Start moving crate " << msg->crate.name << std::endl;
		/*if(crateID.compare(msg->crate.name) == 0){
		 std::cout << "[DEBUG] Moving to new coordinate " << msg->crate.x << "," << msg->crate.y << std::endl;
		 moveToPointService.request.motion.x = msg->crate.x;
		 moveToPointService.request.motion.y = msg->crate.y;
		 deltaRobotClient.call(moveToPointService);
		 }*/
		break;
	case rexos_vision::CrateEvent::type_moved:
		std::cout << "[DEBUG] Moved crate " << msg->crate.name << std::endl;
		/*if(crateID.compare(msg->crate.name) == 0){
		 std::cout << "[DEBUG] Moving to new coordinate " << msg->crate.x << "," << msg->crate.y << std::endl;
		 moveToPointService.request.motion.x = msg->crate.x;
		 moveToPointService.request.motion.y = msg->crate.y;
		 deltaRobotClient.call(moveToPointService);
		 }*/
		break;
	}
}

void FollowNode::run( ){

	std::cout << "Welcome to the followNode. This tool will try to follow a crate that has been scanned before. :)."
			<< std::endl << "A\tAssign crate name to tracker ID" << std::endl << "S\tStop following a crate"
			<< std::endl << "Q\tQuit program" << std::endl << "Enter a key and press the \"Enter\" button" << std::endl;

	ros::Subscriber subscriber = nodeHandle.subscribe(CrateLocatorNodeTopics::CRATE_EVENT, 1000, &FollowNode::callback,
			this);

	while(ros::ok() && topicRunning){
		ros::spinOnce();

		if(crateLocatorClient.call(getCrateService)
				&& getCrateService.response.state != rexos_datatypes::Crate::state_non_existing){
			std::cout << getCrateService.response.crate.x << " " << getCrateService.response.crate.y << std::endl;
			//moveToPointService.request.motion.x = getCrateService.response.crate.x;
			//moveToPointService.request.motion.y = getCrateService.response.crate.y;
			//deltaRobotClient.call(moveToPointService);
		}
	}
}

void FollowNode::inputThreadMethod(FollowNode* that){
	try{
		char key;

		while(that->inputRunning){
			std::cin >> key;
			if(key == 'a' || key == 'A'){
				std::cout << "[DEBUG] Looking for a new crate (first new crate is assigned!)" << std::endl;
				that->updateCrateIDFlag = true;
			} else if(key == 's' || key == 'S'){
				that->crateID = "";
				that->getCrateService.request.name = that->crateID;
				std::cout << "[DEBUG] Deleted crateID. Stopped following." << std::endl;
			} else if(key == 'q' || key == 'Q'){
				that->inputRunning = false;
				that->topicRunning = false;
			}

			std::cout.flush();
			ros::spinOnce();
		}

	} catch(boost::thread_interrupted& ignored){
		// Ignore interrupt and exit thread.
	}
}

/**
 * Main function for the followNode, will start it
 **/
int main(int argc, char* argv[]){
	ros::init(argc, argv, "follow");
	FollowNode crateLocatorNode;
	crateLocatorNode.run();

	return 0;
}
