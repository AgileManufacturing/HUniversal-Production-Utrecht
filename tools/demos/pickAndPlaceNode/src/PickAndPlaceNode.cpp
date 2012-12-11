/**
 * @file PickAndPlaceNode.cpp
 * @brief Listens to the crateEvent topic and guides the deltarobot and gripper
 * @date Created: 2012-11-13
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

/**
 * The constructor
 **/
PickAndPlaceNode::PickAndPlaceNode( ) :
		deltaRobotClient(nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)), crateLocatorClient(nodeHandle.serviceClient<crateLocatorNode::getCrate>(CrateLocatorNodeServices::GET_CRATE)), gripperGripClient(nodeHandle.serviceClient<gripperNode::Grip>(GripperNodeServices::GRIP)), gripperReleaseClient(nodeHandle.serviceClient<gripperNode::Release>(GripperNodeServices::RELEASE)) {
}

/**
 * Moves to a source crate, grabs a ball, lifts it, puts it back into the destination crate and releases the ball
 *
 * @param sourceName Name of the crate containing the ball to move
 * @param sourceIndex Number of the ball in the crate as specified in the technical design
 * @param destinationName Name of the destination crate
 * @param destinationIndex Number of the ball in the crate as specified in the technical design
 **/
bool PickAndPlaceNode::relocateBall(std::string sourceName, int sourceIndex, std::string destinationName, int destinationIndex) {
	// Setup the move to point service
	deltaRobotNode::MoveToPoint moveToPointService;
	moveToPointService.request.motion.maxAcceleration = PickAndPlaceNodeSettings::ACCELERATION;
	crateLocatorNode::getCrate getCrateService;

	// Find location of the index in the crate
	GridCrate4x4MiniBall sourceCrate(sourceName);
	getCrateService.request.name = sourceName;
	if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
		sourceCrate.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
	} else {
		// Starts the while loop and locates the crate again.
		sourceCrate.setCrate(1000, 1000, 0);
		std::cerr << "[ERROR] Failed to call getCrateService for crate " << sourceName << std::endl;
	}

	// Move to (track) the grip point
	DataTypes::Point2D sourceLocation = sourceCrate.getLocation(sourceIndex);
	DataTypes::Point2D sourceCurrentLocation;
	while (sourceCurrentLocation.distance(sourceLocation) > 0.2) {

		if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
			sourceCrate.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
		} else {
			std::cerr << "[ERROR] Failed to call getCrateService for crate " << sourceName << std::endl;
			Utilities::sleep(200);
			continue;
		}
		sourceCurrentLocation = sourceCrate.getLocation(sourceIndex);

		// Move to location of the ball
		moveToPointService.request.motion.x = sourceCurrentLocation.x;
		moveToPointService.request.motion.y = sourceCurrentLocation.y;
		moveToPointService.request.motion.z = PickAndPlaceNodeSettings::Z_MOVING;
		deltaRobotClient.call(moveToPointService);

		if (!moveToPointService.response.succeeded) {
			ROS_WARN("[PICK AND PLACE] Delta robot moveTo failed.");
			Utilities::sleep(200);
			continue;
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

	if (!gripService.response.succeeded) {
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
		// Looks for the crate in while loop
		destinationCrate.setCrate(1000, 1000, 0);
	}
	DataTypes::Point2D destinationLocation = destinationCrate.getLocation(destinationIndex);

	// Move to the drop point
	DataTypes::Point2D destinationCurrentLocation;
	while (destinationCurrentLocation.distance(destinationLocation) > 0.2) {

		if (crateLocatorClient.call(getCrateService) && getCrateService.response.state != DataTypes::Crate::state_non_existing) {
			destinationCrate.setCrate(getCrateService.response.crate.x, getCrateService.response.crate.y, getCrateService.response.crate.angle);
		} else {
			std::cerr << "[ERROR] Failed to call getCrateService for crate " << destinationName << std::endl;
			Utilities::sleep(200);
			continue;
		}
		destinationCurrentLocation = destinationCrate.getLocation(destinationIndex);

		// Move to location of the ball
		moveToPointService.request.motion.x = destinationCurrentLocation.x;
		moveToPointService.request.motion.y = destinationCurrentLocation.y;
		moveToPointService.request.motion.z = -270;
		deltaRobotClient.call(moveToPointService);

		if (!moveToPointService.response.succeeded) {
			ROS_WARN("[PICK AND PLACE] Delta robot moveTo failed.");
			Utilities::sleep(200);
			continue;
		}
		destinationLocation = destinationCurrentLocation;
	}

	// Move down to release the ball
	moveToPointService.request.motion.z = PickAndPlaceNodeSettings::Z_RELEASE;
	deltaRobotClient.call(moveToPointService);

	// Release the ball by releasing the gripper
	gripperNode::Release releaseService;
	gripperReleaseClient.call(releaseService);

	if (!releaseService.response.succeeded) {
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
 * blocking function that contains the main loop: moves beads from one crate to another
 **/
void PickAndPlaceNode::run( ) {
	std::cout << "Welcome to the pick and place demo." << std::endl;

	// Run the demo.
	while (ros::ok()) {
		// Move crate "GC4x4MB_1" to "GC4x4MB_5"
		for (int i = 0; i < 16; i++) {
			relocateBall("GC4x4MB_1", i, "GC4x4MB_5", i);
		}

		// Move crate "GC4x4MB_5" to "GC4x4MB_1"
		for (int i = 0; i < 16; i++) {
			relocateBall("GC4x4MB_5", i, "GC4x4MB_1", i);
		}
	}
}

/**
 * Main that starts the pick and place demo
 **/
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "PickAndPlaceNode");
	PickAndPlaceNode pickAndPlaceNode;
	pickAndPlaceNode.run();

	return 0;
}

/**
 * Switches all inner and outer balls.
 **/
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

/**
 * Switches all inner and outer balls.
 **/
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

/**
 * Switches all inner and outer balls.
 **/
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

/**
 * Switches all inner and outer balls.
 **/
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

/**
 * Switches all inner and outer balls.
 **/
void PickAndPlaceNode::switchDiagonalsWithNonDiagonals(std::string crateA, std::string crateB) {
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
