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
#include <keyboard_control_node/KeyBoardControlNode.h>
#include <ros/ros.h>
#include <rexos_datatypes/InstructionData.h>
#include "rexos_utilities/Utilities.h"

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "KeyBoardControlNode"
// @endcond

using namespace keyboard_control_node;


KeyBoardControlNode::KeyBoardControlNode(std::string blackboardIp) :
	currentXPos(0.0),
	currentYPos(-0.0),
	currentZPos(-200.0),
	maxAcceleration("50.0") {

	ROS_INFO("Constructing");

	//DirectMoveStepsBlackBoard
	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "EQ2", "DirectMoveStepsBlackBoard");

	tcgetattr(KEYBOARDNUMBER, &oldTerminalSettings);
	memcpy(&newTerminalSettings, &oldTerminalSettings, sizeof(struct termios));
	newTerminalSettings.c_lflag &=~ (ICANON | ECHO);

	// Setting a new line, then end of file.
	newTerminalSettings.c_cc[VEOL] = 1;
	newTerminalSettings.c_cc[VEOF] = 2;
	tcsetattr(KEYBOARDNUMBER, TCSANOW, &newTerminalSettings);

	equipletStepBlackboardClient->removeDocuments("");

	ROS_INFO("Reading from keyboard");
	ROS_INFO("Start controlling the robot by pressing WASD keys and Up and Down keys");

	run();
}

KeyBoardControlNode::~KeyBoardControlNode(){
	ROS_INFO("Destructing");
}

void KeyBoardControlNode::run(){
	for(;;){
		// Get the next event from the keyboard.
		if(read(KEYBOARDNUMBER, &inputCharacter, 1) < 0){
			perror("read():");
			exit(1);
		}
		readInputFromKeyBoard(inputCharacter);
	}
}

void KeyBoardControlNode::readInputFromKeyBoard(int inputCharacter){
// Reads keyboard input and does things. Doesnt keep track of actual robot location, just relative. Starts on 0.0, 0.0, 0.0

	// Check which key was pressed.
	switch(inputCharacter){
		case KEYCODE_UP:
			ROS_INFO("PRESSED UP");
			//z + step
			currentZPos = currentZPos + STEP;
			break;

		case KEYCODE_DOWN:
			ROS_INFO("PRESSED DOWN");
			//z - step
			currentZPos = currentZPos - STEP;
		break;

		case KEYCODE_W:
			ROS_INFO("PRESSED W");
			//y + step
			currentYPos = currentYPos + STEP;
		break;

		case KEYCODE_A:
			ROS_INFO("PRESSED A");
			//x - step
			currentXPos = currentXPos - STEP;
		break;

		case KEYCODE_S:
			ROS_INFO("PRESSED S");
			//y - step
			currentYPos = currentYPos - STEP;
			break;

		case KEYCODE_D:
			ROS_INFO("PRESSED D");
			//x + step
			currentXPos = currentXPos + STEP;
			break;

		case KEYCODE_C:
			//calibrateClient.call(calibrateService);
		break;
	}
		writeToBlackBoard(rexos_utilities::doubleToString(currentXPos), rexos_utilities::doubleToString(currentYPos), rexos_utilities::doubleToString(currentZPos), maxAcceleration);
}

void KeyBoardControlNode::writeToBlackBoard(std::string x, std::string y, std::string z, std::string acceleration){
	//Need to include InstructionData for the instructiondata
	//Dont forget to set in package & makelist!

	std::map<std::string, std::string> look_up_parameters;
	std::map<std::string, std::string> payload;

	if(!x.empty()) {
		payload.insert(pair<string, string>("x", x));
	}

	if(!y.empty()) {
		payload.insert(pair<string, string>("y", y));
	}

	if(!z.empty()) {
		payload.insert(pair<string, string>("z", z));
	}

	if(!acceleration.empty()) {
		payload.insert(pair<string, string>("maxAcceleration", acceleration));
	}

	instructionData = new rexos_datatypes::InstructionData("move", "deltarobot", "NULL", 
            look_up_parameters, payload);

	if(equipletStepBlackboardClient->insertDocument(instructionData->toJSONString())) {
		std::cout << "printed: " << instructionData->toJSONString() << "to blackboard." << std::endl;
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

	ros::init(argc, argv, "keyBoardControlNode");
	KeyBoardControlNode keyBoardControlNode("145.89.191.131");

	ros::Rate poll_rate(5);

	while (ros::ok()) {
		poll_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
