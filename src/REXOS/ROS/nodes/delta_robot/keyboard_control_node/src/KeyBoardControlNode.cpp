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
#include "rexos_utilities/Utilities.h"
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/writer.h>
#include <rexos_datatypes/EquipletStep.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "KeyBoardControlNode"
// @endcond

using namespace keyboard_control_node;


KeyBoardControlNode::KeyBoardControlNode(std::string blackboardIp) :
	maxAcceleration(50.0) {

	REXOS_INFO("Constructing");

	//DirectMoveStepsBlackBoard
	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "EQ2", "EquipletStepsBlackBoard");

	tcgetattr(KEYBOARDNUMBER, &oldTerminalSettings);
	memcpy(&newTerminalSettings, &oldTerminalSettings, sizeof(struct termios));
	newTerminalSettings.c_lflag &=~ (ICANON | ECHO);

	// Setting a new line, then end of file.
	newTerminalSettings.c_cc[VEOL] = 1;
	newTerminalSettings.c_cc[VEOF] = 2;
	tcsetattr(KEYBOARDNUMBER, TCSANOW, &newTerminalSettings);

	equipletStepBlackboardClient->removeDocuments("");

	REXOS_INFO("Reading from keyboard");
	REXOS_INFO("Start controlling the robot by pressing WASD keys and Up and Down keys");

	run();
}

KeyBoardControlNode::~KeyBoardControlNode(){
	REXOS_INFO("Destructing");
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
	Vector3 direction;

	// Check which key was pressed.
	switch(inputCharacter){
		case KEYCODE_UP:
			REXOS_INFO("PRESSED UP");
			direction.z+= STEP;
		break;
		case KEYCODE_DOWN:
			REXOS_INFO("PRESSED DOWN");
			direction.z-= STEP;
		break;

		case KEYCODE_W:
			REXOS_INFO("PRESSED W");
			direction.y+= STEP;
		break;

		case KEYCODE_A:
			REXOS_INFO("PRESSED A");
			direction.x-= STEP;
		break;

		case KEYCODE_S:
			REXOS_INFO("PRESSED S");
			direction.y-= STEP;
			break;

		case KEYCODE_D:
			REXOS_INFO("PRESSED D");
			direction.x+= STEP;
			break;

		case KEYCODE_C:
			//calibrateClient.call(calibrateService);
		break;
	}
		writeToBlackBoard(direction, maxAcceleration);
}

void KeyBoardControlNode::writeToBlackBoard(Vector3 direction, double acceleration){
	rexos_datatypes::EquipletStep equipletStep;
	Json::Value instructionData;
	Json::Value moveCommand;
	
	moveCommand["x"] = direction.x;
	moveCommand["y"] = direction.y;
	moveCommand["z"] = direction.z;
	moveCommand["maxAcceleration"] = acceleration;
	
	instructionData["move"] = moveCommand;
	equipletStep.setInstructionData(instructionData);
	
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

	ros::init(argc, argv, "keyBoardControlNode");
	KeyBoardControlNode keyBoardControlNode("145.89.191.131");

	ros::Rate poll_rate(5);

	while (ros::ok()) {
		poll_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
