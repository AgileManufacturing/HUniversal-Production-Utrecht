/**
 * @file DummyModuleNode.cpp
 * @brief A dummy module!
 * @date Created: 2013-03-13
 *
 * @author Arjen van Zanten
 * @author Ammar Abdulamir
 *
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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

#include "dummy_module_node/DummyModuleNode.h"
#include "dummy_module_node/Services.h"

// @cond HIDE_NODE_NAME_BASE_FROM_DOXYGEN
#define NODE_NAME_BASE "dummy_module_node"
// @endcond

DummyModuleNode::DummyModuleNode(int equipletID, int moduleID) :
	rexos_statemachine::ModuleStateMachine(std::string(NODE_NAME_BASE), equipletID, moduleID) {

	std::stringstream stringStream;
	stringStream << NODE_NAME_BASE << "_" << equipletID << "_" << moduleID;
	nodeName = stringStream.str();

	std::cout << "Constructing " << nodeName << std::endl;

	ros::NodeHandle nodeHandle;

	stringStream.clear();
	stringStream.str("");
	stringStream << nodeName << "/" << DummyModuleNodeServices::OUTPUTJSON;
	std::string outputJSONServiceName = stringStream.str();

	outputJSONService = nodeHandle.advertiseService(outputJSONServiceName, &DummyModuleNode::outputJSON, this);
}

bool DummyModuleNode::outputJSON(rexos_std_srvs::Module::Request &req, rexos_std_srvs::Module::Response &res) {
	ROS_INFO("outputJSON called");
	std::cout << std::string(req.json) << std::endl;
	res.succeeded = true;
	res.message = "success!";
	return true;
}

void DummyModuleNode::transitionSetup() {
	ROS_INFO("Setup transition called");
}

void DummyModuleNode::transitionShutdown() {
	ROS_INFO("Shutdown transition called");
}

void DummyModuleNode::transitionStart() {
	ROS_INFO("Start transition called");
}

void DummyModuleNode::transitionStop() {
	ROS_INFO("Stop transition called");
}

DummyModuleNode::~DummyModuleNode() {
	ROS_INFO("Destructing");
}

int main(int argc, char **argv) {
	int equipletID = 0;
	int moduleID = 0;

	if (argc < 3 || !(rexos_utilities::stringToInt(equipletID, argv[1]) == 0 && rexos_utilities::stringToInt(moduleID, argv[2]) == 0)) {
		ROS_INFO("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -1;
	}

	std::stringstream stringStream;
	stringStream << NODE_NAME_BASE << "_" << equipletID << "_" << moduleID;
	std::string nodeName = stringStream.str();
	ros::init(argc, argv, nodeName);

	ROS_INFO("Creating Dummy Module");

	DummyModuleNode dummy(equipletID, moduleID);

	ROS_INFO("Running StateEngine");
	ros::spin();
	return 0;
}
