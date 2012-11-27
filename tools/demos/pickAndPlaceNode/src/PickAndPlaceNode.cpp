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
#include <DataTypes/GridCrate4x4MiniBall.h>



PickAndPlaceNode::PickAndPlaceNode( ) :
		crateID(""), updateCrateIDFlag(false), inputRunning(true), topicRunning(true), deltaRobotClient(nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)), crateLocatorClient(nodeHandle.serviceClient<crateLocatorNode::getCrate>(CrateLocatorNodeServices::GET_CRATE)), gripperGripClient(nodeHandle.serviceClient<gripperNode::Grip>(GripperNodeServices::GRIP)), gripperReleaseClient(nodeHandle.serviceClient<gripperNode::Release>(GripperNodeServices::RELEASE)) {
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
bool PickAndPlaceNode::relocateBall(std::string sourceName, int sourceIndex, std::string destinationName, int destinationIndex) {
	deltaRobotNode::MoveToPoint moveToPointService;
	moveToPointService.request.motion.maxAcceleration = PickAndPlaceNodeSettings::ACCELERATION;
	crateLocatorNode::getCrate getCrateService;

	// Find location of the index in the crate
	GridCrate4x4MiniBall sourceCrate(sourceName);
	getCrateService.request.name = sourceName;
	if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
		sourceCrate.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
	} else {
		std::cerr << "[ERROR] Failed to call getCrateService for crate " << sourceName << std::endl;
		return false;
	}

	DataTypes::Point2D sourceLocation = sourceCrate.getLocation(sourceIndex);
	// Move to the grip point
	DataTypes::Point2D sourceCurrentLocation;
	while (sourceCurrentLocation.distance(sourceLocation) > 0.5) {

		if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
			sourceCrate.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
		} else {
			std::cerr << "[ERROR] Failed to call getCrateService for crate " << sourceName << std::endl;
			return false;
		}
		sourceCurrentLocation = sourceCrate.getLocation(sourceIndex);

		// Move to location of the ball
		moveToPointService.request.motion.x = sourceCurrentLocation.x;
		moveToPointService.request.motion.y = sourceCurrentLocation.y;
		moveToPointService.request.motion.z = PickAndPlaceNodeSettings::Z_MOVING;
		deltaRobotClient.call(moveToPointService);

		if (!moveToPointService.response.succeeded) {
			ROS_WARN("[PICK AND PLACE] Delta robot moveTo failed.");
			return false;
		}
		sourceLocation = sourceCurrentLocation;
	}

	// Move down to touch the ball
	moveToPointService.request.motion.z = PickAndPlaceNodeSettings::Z_GRAB;
	deltaRobotClient.call(moveToPointService);

	// Grab the ball by enabling the gripper and wait 200ms for the vacuum to build up.
	gripperNode::Grip gripService;
	gripperGripClient.call(gripService);
	Utilities::sleep(200);

	if(!gripService.response.succeeded){
		// Warn only once
		ROS_WARN("[PICK AND PLACE] Gripper grab failed.");
	}

	while (!gripService.response.succeeded) {
		// Gripper failed... stop the action for now.
		gripperGripClient.call(gripService);
		Utilities::sleep(200);
	}

	// Move up get the ball out of the crate
	moveToPointService.request.motion.z = PickAndPlaceNodeSettings::Z_MOVING;
	deltaRobotClient.call(moveToPointService);

	// Find location of the index in the destination crate
	GridCrate4x4MiniBall destinationCrate(destinationName);
	getCrateService.request.name = destinationName;
	if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
		destinationCrate.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
	} else {
		std::cerr << "[ERROR] Failed to call getCrateService for crate " << destinationName << std::endl;
		return false;
	}
	DataTypes::Point2D destinationLocation = destinationCrate.getLocation(destinationIndex);

	// Move to the drop point
	DataTypes::Point2D destinationCurrentLocation;
	while (destinationCurrentLocation.distance(destinationLocation) > 0.5) {

		if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
			destinationCrate.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
		} else {
			std::cerr << "[ERROR] Failed to call getCrateService for crate " << destinationName << std::endl;
			return false;
		}
		destinationCurrentLocation = destinationCrate.getLocation(destinationIndex);

		// Move to location of the ball
		moveToPointService.request.motion.x = destinationCurrentLocation.x;
		moveToPointService.request.motion.y = destinationCurrentLocation.y;
		moveToPointService.request.motion.z = -270;
		deltaRobotClient.call(moveToPointService);

		if (!moveToPointService.response.succeeded) {
			ROS_WARN("[PICK AND PLACE] Delta robot moveTo failed.");
			return false;
		}
		destinationLocation = destinationCurrentLocation;
	}

	moveToPointService.request.motion.x = destinationLocation.x;
	moveToPointService.request.motion.y = destinationLocation.y;
	deltaRobotClient.call(moveToPointService);

	// Move down to release the ball
	moveToPointService.request.motion.z = PickAndPlaceNodeSettings::Z_RELEASE;
	deltaRobotClient.call(moveToPointService);

	// Release the ball by releasing the gripper
	gripperNode::Release releaseService;
	gripperReleaseClient.call(releaseService);

	if(!releaseService.response.succeeded){
		// Warn only once
		ROS_WARN("[PICK AND PLACE] Gripper release failed.");
	}

	while (!releaseService.response.succeeded) {
		// Gripper failed... stop the action for now.
		gripperReleaseClient.call(releaseService);
		Utilities::sleep(200);
	}

	// Move up to get into the start position
	moveToPointService.request.motion.z = PickAndPlaceNodeSettings::Z_MOVING;
	deltaRobotClient.call(moveToPointService);

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

	//DataTypes::Point2D sourceLocation(0.0, 0.0);
	//DataTypes::Point2D destinationLocation(30.0, 30.0);

	//grabBall(sourceLocation, destinationLocation);

	bool direction = false;

	while (ros::ok() && topicRunning) {
		ros::spinOnce();
		crateLocatorNode::getCrate getCrateService;

		GridCrate4x4MiniBall crateSource("GC4x4MB_3");
		GridCrate4x4MiniBall crateDestination("GC4x4MB_4");

		getCrateService.request.name = "GC4x4MB_3";
		if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
			std::cout << getCrateService.response.crate.x << " " << getCrateService.response.crate.y << std::endl;
			crateSource.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);

			std::cout << "Location 1" << crateSource.getLocation(1) << std::endl;
		} else {
			std::cerr << "[ERROR] Failed to call getCrateService!" << std::endl;
			continue; //exit(1);
		}

		// TODO: remove
		DataTypes::Point2D center(getCrateService.response.crate.x, getCrateService.response.crate.y);
		DataTypes::Point2D bead1 = crateSource.getLocation(6);
		std::cout << "center\t" << center << std::endl;
		std::cout << "Location 1\t" << bead1 << std::endl;
		std::cout << "Distance\t" << center.distance(crateSource.getLocation(1)) << std::endl;
		std::cout << "Diff\t" << (center - crateSource.getLocation(1)) << std::endl;
		// ENDTODO

		getCrateService.request.name = "GC4x4MB_4";
		if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
			std::cout << getCrateService.response.crate.x << " " << getCrateService.response.crate.y << std::endl;
			crateDestination.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
		} else {
			std::cerr << "[ERROR] Failed to call getCrateService!" << std::endl;
			continue; //exit(1);
		}

		// Move to location of the ball
		deltaRobotNode::MoveToPoint moveToPointService;
		moveToPointService.request.motion.maxAcceleration = PickAndPlaceNodeSettings::ACCELERATION;
		std::cout << "bead1\t" << bead1 << std::endl;
		moveToPointService.request.motion.x = bead1.x;
		moveToPointService.request.motion.y = bead1.y;
		moveToPointService.request.motion.z = -270;
		//deltaRobotClient.call(moveToPointService);

		moveToPointService.request.motion.x = bead1.x;
		moveToPointService.request.motion.y = bead1.y;
		moveToPointService.request.motion.z = -281.5;
		//deltaRobotClient.call(moveToPointService);
		if (!direction) {
			relocateBall("GC4x4MB_3",1, "GC4x4MB_4",1);
		} else {
			relocateBall(crateDestination.getName(),1, crateSource.getName(),1);
		}

		direction = !direction;
	}

	// Release the ball by releasing the gripper
	gripperNode::Release releaseService;
	gripperReleaseClient.call(releaseService);
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

void PickAndPlaceNode::switchInnerOuterBalls(std::string crateA) {
	relocateBall(crateA, 6, crateA, 2);
	relocateBall(crateA, 10, crateA, 11);
	relocateBall(crateA, 9, crateA, 13);
	relocateBall(crateA, 5, crateA, 4);

	relocateBall(crateA, 0, crateA, 5);
	relocateBall(crateA, 3, crateA, 6);
	relocateBall(crateA, 15, crateA, 10);
	relocateBall(crateA, 12, crateA, 9);

	relocateBall(crateA, 13, crateA, 12);
	relocateBall(crateA, 4, crateA, 0);
	relocateBall(crateA, 2, crateA, 3);
	relocateBall(crateA, 11, crateA, 15);
}

void PickAndPlaceNode::switchInnerOuterBalls2(std::string crateA) {
	relocateBall(crateA, 15, crateA, 14);
	relocateBall(crateA, 12, crateA, 8);
	relocateBall(crateA, 0, crateA, 1);
	relocateBall(crateA, 3, crateA, 7);

	relocateBall(crateA, 6, crateA, 3);
	relocateBall(crateA, 10, crateA, 15);
	relocateBall(crateA, 9, crateA, 12);
	relocateBall(crateA, 5, crateA, 0);

	relocateBall(crateA, 1, crateA, 5);
	relocateBall(crateA, 7, crateA, 6);
	relocateBall(crateA, 14, crateA, 10);
	relocateBall(crateA, 8, crateA, 9);
}

void PickAndPlaceNode::switchInnerOuterBalls3(std::string crateA) {
	relocateBall(crateA, 9, crateA, 13);
	relocateBall(crateA, 12, crateA, 9);
	relocateBall(crateA, 13, crateA, 12);

	relocateBall(crateA, 5, crateA, 4);
	relocateBall(crateA, 0, crateA, 5);
	relocateBall(crateA, 4, crateA, 0);

	relocateBall(crateA, 6, crateA, 2);
	relocateBall(crateA, 3, crateA, 6);
	relocateBall(crateA, 2, crateA, 3);

	relocateBall(crateA, 10, crateA, 11);
	relocateBall(crateA, 15, crateA, 10);
	relocateBall(crateA, 11, crateA, 15);
}

void PickAndPlaceNode::switchInnerOuterBalls4(std::string crateA) {
	relocateBall(crateA, 15, crateA, 14);
	relocateBall(crateA, 10, crateA, 15);
	relocateBall(crateA, 14, crateA, 10);

	relocateBall(crateA, 3, crateA, 7);
	relocateBall(crateA, 6, crateA, 3);
	relocateBall(crateA, 7, crateA, 6);

	relocateBall(crateA, 0, crateA, 1);
	relocateBall(crateA, 5, crateA, 0);
	relocateBall(crateA, 1, crateA, 5);

	relocateBall(crateA, 12, crateA, 8);
	relocateBall(crateA, 9, crateA, 12);
	relocateBall(crateA, 8, crateA, 9);
}

void PickAndPlaceNode::switchDiagonalsWithNonDiagonals(std::string crateA, std::string crateB){
	relocateBall(crateA, 0, crateB, 0);
	relocateBall(crateA, 5, crateB, 5);
	relocateBall(crateA, 6, crateB, 6);
	relocateBall(crateA, 10, crateB, 10);
	relocateBall(crateA, 9, crateB, 9);
	relocateBall(crateA, 12, crateB, 12);
	relocateBall(crateA, 15, crateB, 15);
	relocateBall(crateA, 3, crateB, 3);

	relocateBall(crateB, 2, crateA, 2);
	relocateBall(crateB, 1, crateA, 1);
	relocateBall(crateB, 4, crateA, 4);
	relocateBall(crateB, 8, crateA, 8);
	relocateBall(crateB, 13, crateA, 13);
	relocateBall(crateB, 14, crateA, 14);
	relocateBall(crateB, 11, crateA, 11);
	relocateBall(crateB, 7, crateA, 7);
}
