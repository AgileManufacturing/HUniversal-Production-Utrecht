/**
 * @file PickAndPlaceNode.cpp
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
 
#include <PickAndPlaceNode/PickAndPlaceNode.h>
#include <PickAndPlaceNode/PickAndPlaceNodeSettings.h>
#include <CrateLocatorNode/Topics.h>
#include <CrateLocatorNode/Services.h>
#include <DataTypes/Crate.h>
#include <DeltaRobotNode/Services.h>
#include <Vision/CrateTracker.h>
#include <GripperNode/Services.h>
#include <Utilities/Utilities.h>

PickAndPlaceNode::PickAndPlaceNode( ) :
		crateID(""), updateCrateIDFlag(false), inputRunning(true), topicRunning(true),
		deltaRobotClient(nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)),
		crateLocatorClient(nodeHandle.serviceClient<crateLocatorNode::getCrate>(CrateLocatorNodeServices::GET_CRATE)),
		gripperGripClient(nodeHandle.serviceClient<gripperNode::Grip>(GripperNodeServices::GRIP)),
		gripperReleaseClient(nodeHandle.serviceClient<gripperNode::Release>(GripperNodeServices::RELEASE))
		{
	inputThread = new boost::thread(inputThreadMethod, this);

	// Z -281 voor de bal
}

PickAndPlaceNode::~PickAndPlaceNode( ) {
	inputRunning = false;
	topicRunning = false;
	inputThread->interrupt();
}

/**
 * Initial test function. Moves to a ball, grabs it, lifts it, puts it back into the crate and releases it.
 */
bool PickAndPlaceNode::grabBall(DataTypes::Point2D sourceLocation, DataTypes::Point2D destinationLocation) {
	deltaRobotNode::MoveToPoint moveToPointService;
	moveToPointService.request.motion.maxAcceleration = PickAndPlaceNodeSettings::ACCELERATION;

	// Move to location of the ball
	moveToPointService.request.motion.x = sourceLocation.x;
	moveToPointService.request.motion.y = sourceLocation.y;
	moveToPointService.request.motion.z = -270;
	deltaRobotClient.call(moveToPointService);

	if(!moveToPointService.response.succeeded){
		ROS_WARN("[PICK AND PLACE] Delta robot moveTo failed.");
		return false;
	}

	// Move down to touch the ball
	moveToPointService.request.motion.z = -281;
	deltaRobotClient.call(moveToPointService);

	// Grab the ball by enabling the gripper and wait 200ms for the vacuum to build up.
	gripperNode::Grip gripService;
	gripperGripClient.call(gripService);
	Utilities::sleep(200);

	// Move up get the ball out of the crate
	moveToPointService.request.motion.z = -260;
	deltaRobotClient.call(moveToPointService);

	if(!gripService.response.succeeded){
		// Gripper failed... stop the action for now.
		ROS_WARN("[PICK AND PLACE] Gripper grab failed.");
		return false;
	}

	// Move to the drop point
	moveToPointService.request.motion.x = destinationLocation.x;
	moveToPointService.request.motion.y = destinationLocation.y;
	deltaRobotClient.call(moveToPointService);

	// Move down to release the ball
	moveToPointService.request.motion.z = -275;
	deltaRobotClient.call(moveToPointService);

	// Release the ball by releasing the gripper
	gripperNode::Release releaseService;
	gripperReleaseClient.call(releaseService);

	// Move up to get into the start position
	moveToPointService.request.motion.z = -270;
	deltaRobotClient.call(moveToPointService);

	if(!releaseService.response.succeeded){
		// Gripper failed... stop the action for now.
		ROS_ERROR("[PICK AND PLACE] Gripper release failed.");
		return false;
	}

	return true;
}

/**
 * Function to act on received Crate Event messages
 *
 * @param msg CrateEventMsg pointer.
 **/
void PickAndPlaceNode::callback(const crateLocatorNode::CrateEventMsg::ConstPtr& msg) {
	switch (msg->event) {
	case Vision::CrateEvent::type_in:
		std::cout << "[DEBUG] New crate " << msg->crate.name << "found!" << std::endl;
		if (updateCrateIDFlag) {
			crateID = msg->crate.name;
			std::cout << "[DEBUG] Tracking new crate " << crateID << std::endl;
			updateCrateIDFlag = false;
		}

		// Move to the crate!
		/*if (crateID.compare(msg->crate.name) == 0) {
		 std::cout << "[DEBUG] Moving to new coordinate " << msg->crate.x << "," << msg->crate.y << std::endl;
		 moveToPointService.request.motion.x = msg->crate.x;
		 moveToPointService.request.motion.y = msg->crate.y;
		 deltaRobotClient.call(moveToPointService);
		 }*/
		break;
	case Vision::CrateEvent::type_out:
		std::cout << "[DEBUG] Deleted crate " << msg->crate.name << std::endl;
		//if (crateID.compare(msg->crate.name) == 0) {
		//	crateID = "";
		//	std::cout << "[DEBUG] Lost crate " << msg->crate.name << ". Stopped the follow process " << std::endl;
		//}
		break;
	case Vision::CrateEvent::type_moving:
		std::cout << "[DEBUG] Start moving crate " << msg->crate.name << std::endl;
		/*if (crateID.compare(msg->crate.name) == 0) {
		 std::cout << "[DEBUG] Moving to new coordinate " << msg->crate.x << "," << msg->crate.y << std::endl;
		 moveToPointService.request.motion.x = msg->crate.x;
		 moveToPointService.request.motion.y = msg->crate.y;
		 deltaRobotClient.call(moveToPointService);
		 }*/
		break;
	case Vision::CrateEvent::type_moved:
		std::cout << "[DEBUG] Moved crate " << msg->crate.name << std::endl;
		if (crateID.compare(msg->crate.name) == 0) {
			std::cout << "[DEBUG] Crate '" << msg->crate.name << "' stopped moving" << std::endl;
			deltaRobotNode::MoveToPoint moveToPointService;
			moveToPointService.request.motion.x = msg->crate.x;
			moveToPointService.request.motion.y = msg->crate.y;
			deltaRobotClient.call(moveToPointService);
		}
		break;
	}
}

void PickAndPlaceNode::run( ) {

	std::cout << "Welcome to the followNode. This tool will try to follow a crate that has been scanned before. :)." << std::endl << "A\tAssign crate name to tracker ID" << std::endl << "S\tStop following a crate" << std::endl << "Q\tQuit program" << std::endl << "Enter a key and press the \"Enter\" button" << std::endl;

	ros::Subscriber subscriber = nodeHandle.subscribe(CrateLocatorNodeTopics::CRATE_EVENT, 1000, &PickAndPlaceNode::callback, this);

	DataTypes::Point2D sourceLocation(0.0, 0.0);
	DataTypes::Point2D destinationLocation(30.0, 30.0);

	grabBall(sourceLocation, destinationLocation);

	/*while (ros::ok() && topicRunning) {
		ros::spinOnce();

		crateLocatorNode::getCrate getCrateService;
		getCrateService.request.name = crateID;
		if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
			std::cout << getCrateService.response.crate.x << " " << getCrateService.response.crate.y << std::endl;
			deltaRobotNode::MoveToPoint moveToPointService;
			moveToPointService.request.motion.x = getCrateService.response.crate.x;
			moveToPointService.request.motion.y = getCrateService.response.crate.y;
			deltaRobotClient.call(moveToPointService);
		}
	}*/
}

void PickAndPlaceNode::inputThreadMethod(PickAndPlaceNode* that) {
	try {
		char key;

		while (that->inputRunning) {
			std::cin >> key;
			if (key == 'a' || key == 'A') {
				std::cout << "[DEBUG] Looking for a new crate (first new crate is assigned!)" << std::endl;
				that->updateCrateIDFlag = true;
			} else if (key == 's' || key == 'S') {
				that->crateID = "";
				std::cout << "[DEBUG] Deleted crateID. Stopped following." << std::endl;
			} else if (key == 'q' || key == 'Q') {
				that->inputRunning = false;
				that->topicRunning = false;
			}

			std::cout.flush();
			ros::spinOnce();
		}

	} catch (boost::thread_interrupted& ignored) {
		// Ignore interrupt and exit thread.
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "PickAndPlaceNode");
	PickAndPlaceNode pickAndPlaceNode;
	pickAndPlaceNode.run();

	return 0;
}
