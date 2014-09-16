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
#include <rexos_datatypes/InstructionData.h>
#include <rexos_datatypes/EquipletStep.h>
#include "rexos_utilities/Utilities.h"

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "part_follow_node"
// @endcond

using namespace part_follow_node;


PartFollowNode::PartFollowNode(std::string blackboardIp) :
	maxAcceleration("5.0") {

	REXOS_INFO("Constructing");

	//DirectMoveStepsBlackBoard
	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "EQ2", "EquipletStepsBlackBoard");


	equipletStepBlackboardClient->removeDocuments("");
	REXOS_INFO("A");
}

PartFollowNode::~PartFollowNode(){
	REXOS_INFO("Destructing");
}

void PartFollowNode::run(){
	REXOS_INFO("B");
	ros::Rate rate(1);

	while (ros::ok()) {
	REXOS_INFO("C");
		writeToBlackBoard(maxAcceleration);
		rate.sleep();
		ros::spinOnce();
	}	
}

void PartFollowNode::writeToBlackBoard(std::string acceleration){
	REXOS_INFO("D");

	//Need to include InstructionData for the instructiondata
	//Dont forget to set in package & makelist!

	std::map<std::string, std::string> look_up_parameters;
	std::map<std::string, std::string> payload;

	payload.insert(pair<string, string>("x", "0.0"));
	payload.insert(pair<string, string>("y", "0.0"));
	payload.insert(pair<string, string>("z", "-335"));

	REXOS_INFO("E");
	if(!acceleration.empty()) {
		payload.insert(pair<string, string>("maxAcceleration", acceleration));
	}
	look_up_parameters.insert(pair<string, string>("ID", "GC4x4MB_2"));
	instructionData = new rexos_datatypes::InstructionData("move", "deltarobot", "FIND_ID", 
            look_up_parameters, payload);

	REXOS_INFO("F");
    std::stringstream ss;
        ss << "{ ";
        ss << "\"serviceStepID\" : \"" << "1" << "\", ";
        ss << "\"nextStep\" : \"" << "NULL" << "\", ";
        ss << "\"moduleId\" : \"" << "1" << "\", ";
        ss << "\"instructionData\" : " << instructionData->toJSONString() << ", ";
        ss << "\"status\" : \"WAITING\" ";
        ss << " } ";
	REXOS_INFO(ss.str().c_str());
    rexos_datatypes::EquipletStep step = rexos_datatypes::EquipletStep(libjson::parse(ss.str()));    


	REXOS_INFO("G");
	step.setInstructionData(*instructionData);

	if(equipletStepBlackboardClient->insertDocument(step.toJSONString())) {
		REXOS_INFO_STREAM("printed: " << step.toJSONString() << "to blackboard." << std::endl);
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
