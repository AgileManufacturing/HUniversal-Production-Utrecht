/**
 * @file KeyBoardControlNode.cpp
 * @brief Control the robot with the WASD UP and Down keys.
 * @date Created: 2012-10-05
 *
 * @author Dick van der Steen
 * @author Dennis Koole
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

#include <iostream>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <cstdlib>
#include <part_follow_node/PartFollowNode.h>
#include <ros/ros.h>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/writer.h>
#include <rexos_datatypes/HardwareStep.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "part_follow_node"
// @endcond

using namespace part_follow_node;


PartFollowNode::PartFollowNode(std::string blackboardIp) :
	maxAcceleration(5.0) {

	REXOS_INFO("Constructing");

	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "EQ2", "HardwareStepsBlackBoard");
	//equipletStepBlackboardClient->removeDocuments("");
}

PartFollowNode::~PartFollowNode(){
	REXOS_INFO("Destructing");
}

void PartFollowNode::run(){
	ros::Rate rate(1);

	while (ros::ok()) {
		writeToBlackBoard(maxAcceleration);
		rate.sleep();
		ros::spinOnce();
	}
}

void PartFollowNode::writeToBlackBoard(double acceleration){
	rexos_datatypes::HardwareStep equipletStep;
	Json::Value instructionData;
	Json::Value moveCommand;
	
	moveCommand["x"] = 0;
	moveCommand["y"] = 0;
	moveCommand["z"] = 30;
	moveCommand["maxAcceleration"] = acceleration;
	
	instructionData["move"] = moveCommand;
	equipletStep.setInstructionData(instructionData);
	
	rexos_datatypes::OriginPlacement originPlacement = equipletStep.getOriginPlacement();
	originPlacement.setOriginPlacementType(rexos_datatypes::OriginPlacement::RELATIVE_TO_IDENTIFIER);
	Json::Value parameters;
	parameters["identifier"] = "GC4x4MB_2";
	originPlacement.setParameters(parameters);
	
	Json::StyledWriter writer;
	if(equipletStepBlackboardClient->insertDocument(writer.write(equipletStep.toJSON()))) {
		REXOS_INFO_STREAM("printed: " << equipletStep.toJSON() << "to blackboard.");
	}
}

/**
* Starting method for the KeyBoardControlNode.
*
* @param argc Argument count.
* @param argv Argument vector.
* 
* @return 1 on keyboard read error.
**/
int main(int argc, char** argv){
	// Ros init.
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	PartFollowNode pfn("145.89.191.131");

	pfn.run();

	return 0;
}
