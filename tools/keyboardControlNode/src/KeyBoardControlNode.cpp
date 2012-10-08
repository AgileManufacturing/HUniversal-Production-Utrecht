/**
* @file KeyBoardControlNode.cpp
* @brief Control the robot with the WASD UP and Down keys.
* @date Created: 2012-10-05
*
* @author Dick van der Steen
* @author Dennis Koole
*
* @section LICENSE
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
#include "ros/ros.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/MoveToRelativePoint.h"
#include "deltaRobotNode/Motion.h"
#include "deltaRobotNode/Calibrate.h"
#include "DeltaRobotNode/Services.h"

#define NODE_NAME "KeyBoardControlNode"

// Keycodes
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_C 0x63

// Keyboard number
int kfd =0; 

struct termios cooked, raw;

/**
* Release keyboard safely when Ctrl+C is pressed
*
* @param sig The signal received from the Linux OS
**/
void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}
 
int main(int argc, char** argv) {
	// Ros init
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	// Getting MovePath Services
    ros::ServiceClient deltaRobotClient = nodeHandle.serviceClient<deltaRobotNode::MoveToRelativePoint>(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT);
    deltaRobotNode::MoveToRelativePoint moveToRelativePointService;

    // Getting Calibrate Services
    ros::ServiceClient calibrateClient = nodeHandle.serviceClient<deltaRobotNode::Calibrate>(DeltaRobotNodeServices::CALIBRATE);
    deltaRobotNode::Calibrate calibrateService;
    
	// Initing the keyboard read and setting up clean shutdown
	signal(SIGINT, quit);
	char c;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);

	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
  	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	// The speed in mm per second
	double speed = 100.0;
	
	// The step we take in mm 
	double step = 1.0;

	ROS_INFO("Reading from keyboard");
	ROS_INFO("Start controlling the robot by pressing WASD keys and Up and Down keys");

	deltaRobotNode::Motion motion;
	for(;;) {
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0) {
		  perror("read():");
		  exit(0);
		}

		moveToRelativePointService.request.motion.x = 0;
		moveToRelativePointService.request.motion.y = 0;
		moveToRelativePointService.request.motion.z = 0;
		moveToRelativePointService.request.motion.speed = speed;

		// Check which key was pressed
		switch(c) {
			case KEYCODE_UP:
				ROS_INFO("PRESSED UP");
				moveToRelativePointService.request.motion.z = step;
				deltaRobotClient.call(moveToRelativePointService);
				break;
			case KEYCODE_DOWN:
				ROS_INFO("PRESSED DOWN");
				moveToRelativePointService.request.motion.z = -step;
				deltaRobotClient.call(moveToRelativePointService);
				break;
			case KEYCODE_W:
				ROS_INFO("PRESSED W");
				moveToRelativePointService.request.motion.y = step;
				deltaRobotClient.call(moveToRelativePointService);
				break;
			case KEYCODE_A:
			  	ROS_INFO("PRESSED A");
				moveToRelativePointService.request.motion.x = -step;
				deltaRobotClient.call(moveToRelativePointService);
			  	break;
			case KEYCODE_S:
				ROS_INFO("PRESSED S");
				moveToRelativePointService.request.motion.y = -step;
				deltaRobotClient.call(moveToRelativePointService);
			  	break;
			case KEYCODE_D:
			  	ROS_INFO("PRESSED D");
			  	moveToRelativePointService.request.motion.x = step;
				deltaRobotClient.call(moveToRelativePointService);
				break;
			case KEYCODE_C:
				calibrateClient.call(calibrateService);
				break;
		}
		ros::spinOnce();
	}
	return 0;
}