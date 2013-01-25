/**
 * @file FollowNode.h
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

#pragma once

#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>

#include <crate_locator_node/CrateEventMsg.h>
#include <crate_locator_node/getCrate.h>

/**
 * Followes a crate with the deltarobot
 **/
class FollowNode{
public:
	/**
	 * the constructor
	 */
	FollowNode();
	/**
	 * the destructor
	 */
	~FollowNode();

	/**
	 * blocking function that contains the main loop: take frame, detect crates, send event. this function ends when ros receives a ^c
	 */
	void run();

	/**
	 * Callback for when the cratelocated finds a crate
	 **/
	void callback(const crate_locator_node::CrateEventMsg::ConstPtr& msg);

	/**
	 * Thread that accepts input from the keyboard
	 **/
	static void inputThreadMethod(FollowNode* that);

private:
	/** 
	 * the nodehandle, used to create subscribers and publishers
	 **/
	ros::NodeHandle nodeHandle;

	/**
	 * Identifier for the crate
	 **/
	std::string crateID;

	/**
	 * Flag used to indicate that the crate needs and update
	 **/
	bool updateCrateIDFlag;

	/**
	 * If the input thread is running
	 **/ 
	bool inputRunning;

	/**
	 * If the topic is running
	 **/
	bool topicRunning;

	/**
	 * Thread for the keyboard input
	 **/
	boost::thread* inputThread;

	/**
	 * Client for calling the deltarobot
	 **/
	ros::ServiceClient deltaRobotClient;

	/**
	 * Client for the crate located
	 **/
	ros::ServiceClient crateLocatorClient;

	/**
	 * Contains the request and response for the crateLocator
	 **/
	crate_locator_node::getCrate getCrateService;
};
