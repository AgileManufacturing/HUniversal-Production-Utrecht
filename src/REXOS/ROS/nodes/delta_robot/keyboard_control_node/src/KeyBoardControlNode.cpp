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
#include "keyboard_control_node/KeyBoardControlNode.h"
#include "ros/ros.h"
#include "delta_robot_node/Motion.h"

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "KeyBoardControlNode"
// @endcond

using namespace keyboard_control_node;

/**
 * Release keyboard safely when Ctrl+C is pressed.
 *
 * @param sig The signal received from the Linux OS.
 **/
void KeyBoardControlNode::quit(int sig){
	//tcsetattr(keyboardNumber, TCSANOW, &oldTerminalSettings);
	ROS_INFO("Lets quit.");
	exit(0);
}

KeyBoardControlNode::KeyBoardControlNode(std::string blackboardIp){
	ROS_INFO("Constructing");

	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "EQ1", "EquipletStepsBlackBoard");

	// Initing the keyboard read and setting up clean shutdown.
	signal(SIGINT, &KeyBoardControlNode::quit);

	tcgetattr(keyboardNumber, &oldTerminalSettings);
	memcpy(&newTerminalSettings, &oldTerminalSettings, sizeof(struct termios));
	newTerminalSettings.c_lflag &=~ (ICANON | ECHO);

	// Setting a new line, then end of file.
	newTerminalSettings.c_cc[VEOL] = 1;
	newTerminalSettings.c_cc[VEOF] = 2;
	tcsetattr(keyboardNumber, TCSANOW, &newTerminalSettings);

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
		if(read(keyboardNumber, &inputCharacter, 1) < 0){
			perror("read():");
			exit(1);
		}
		readInputFromKeyBoard(inputCharacter);
	}
}

void KeyBoardControlNode::readInputFromKeyBoard(int inputCharacter){


	/*	moveToRelativePointService.request.motion.x = 0;
		moveToRelativePointService.request.motion.y = 0;
		moveToRelativePointService.request.motion.z = 0;
		moveToRelativePointService.request.motion.maxAcceleration = maxAcceleration; */

	// Check which key was pressed.
	switch(inputCharacter){
		case KEYCODE_UP:
			ROS_INFO("PRESSED UP");
			//moveToRelativePointService.request.motion.z = step;
			////deltaRobotClient.call(moveToRelativePointService);
		break;

		case KEYCODE_DOWN:
			ROS_INFO("PRESSED DOWN");
			//moveToRelativePointService.request.motion.z = -step;
			//deltaRobotClient.call(moveToRelativePointService);
		break;

		case KEYCODE_W:
			ROS_INFO("PRESSED W");
			//moveToRelativePointService.request.motion.y = step;
			//deltaRobotClient.call(moveToRelativePointService);
		break;

		case KEYCODE_A:
			ROS_INFO("PRESSED A");
			//moveToRelativePointService.request.motion.x = -step;
			//deltaRobotClient.call(moveToRelativePointService);
		break;

		case KEYCODE_S:
			ROS_INFO("PRESSED S");
			//moveToRelativePointService.request.motion.y = -step;
			//deltaRobotClient.call(moveToRelativePointService);
		break;

		case KEYCODE_D:
			ROS_INFO("PRESSED D");
			////moveToRelativePointService.request.motion.x = step;
			//deltaRobotClient.call(moveToRelativePointService);
		break;

		case KEYCODE_C:
			//calibrateClient.call(calibrateService);
		break;
	}
}

void KeyBoardControlNode::writeToBlackBoard(delta_robot_node::Motion){
	equipletStepBlackboardClient->insertDocument("('_id' : ObjectId('52417406e4b031d5f036416e'),"
               "'instructionData' : {""
                              "'command' : 'move',""
                              "'destination" : 'deltarobot',"
                              "'look_up' : 'FIND_ID',"
                              "'look_up_parameters' : {"
                                            "'ID' : 'RELATIVE-TO-PLACEHOLDER'"
                              "},"
                              "'payload' : {"
                                            "'z' : 25,"
                                            "'maxAcceleration' : 50"
                              "}"
               "},"
               "'moduleId' : 1,"
               "'nextStep' : null,"
               "'serviceStepID' : ObjectId('52417406e4b031d5f036416c'),"
               "'status' : 'DONE',"
               "'statusData' : {"
               "},"
               "'timeData' : {"
                              "'duration' : 6"
               "}"
		"}");	

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
