/**
 * @file PickAndPlaceNode.h
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

#pragma once

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <crateLocatorNode/CrateEventMsg.h>
#include <boost/thread.hpp>
#include <deltaRobotNode/MoveToPoint.h>
#include <deltaRobotNode/MovePath.h>
#include <crateLocatorNode/getCrate.h>
#include <gripperNode/Grip.h>
#include <gripperNode/Release.h>
#include <DataTypes/Point2D.h>

/**
 * The PickAndPlaceNode is a demonstrator for basic pick and place actions.
 **/
class PickAndPlaceNode {
public:

	PickAndPlaceNode( );
	void run( );
	bool relocateBall(std::string sourceName, int sourceIndex, std::string destinationName, int destinationIndex);

private:
	void switchInnerOuterBalls(std::string crateA);
	void switchInnerOuterBalls2(std::string crateA);
	void switchInnerOuterBalls3(std::string crateA);
	void switchInnerOuterBalls4(std::string crateA);
	void switchDiagonalsWithNonDiagonals(std::string crateA, std::string crateB);

	/**
	 * @var ros::NodeHandle nodeHandle
	 * Node handle used for communication with other nodes
	 **/
	ros::NodeHandle nodeHandle;

	/**
	 * @var ros::ServiceClient deltaRobotClient
	 * Service client to call the delta robot movetoPoint service
	 **/
	ros::ServiceClient deltaRobotClient;

	/**
	 * @var ros::ServiceClient crateLocatorClient
	 * Service client to call the getCrateService
	 **/
	ros::ServiceClient crateLocatorClient;

	/**
	 * @var ros::ServiceClient gripperGripClient
	 * Service client to call the grip service
	 **/
	ros::ServiceClient gripperGripClient;

	/**
	 * @var ros::ServiceClient gripperReleaseClient
	 * Service client to call the release service
	 **/
	ros::ServiceClient gripperReleaseClient;
};
