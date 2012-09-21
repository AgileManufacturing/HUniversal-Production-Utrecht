#include <iostream>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <cstdlib>
#include "ros/ros.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/Motion.h"


#define NODE_NAME "KeyBoardControlNode"

// Keycodes
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

int kfd =0; // Keyboard number
struct termios cooked, raw;

/**
*
*
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
    ros::ServiceClient deltaRobotClient = nodeHandle.serviceClient<deltaRobotNode::MovePath>("movePath");
    deltaRobotNode::MovePath movePathService;
    
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

	ROS_INFO("Reading from keyboard");
	ROS_INFO("Start controlling the robot by pressing WASD keys");

	deltaRobotNode::Motion motion;
	for(;;) {
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0) {
		  perror("read():");
		  exit(0);
		}

		// Check which key was pressed
		switch(c) {
			case KEYCODE_LEFT:
				std::cout << "Left";
 				break;
			case KEYCODE_RIGHT:
				std::cout << "Right";
				break;
			case KEYCODE_UP:
				std::cout << "Up";
				break;
			case KEYCODE_DOWN:
				std::cout << "Down";
				break;
			case KEYCODE_W:
				std::cout << "PRESSED W " << std::endl;
				break;
			case KEYCODE_A:
			  	std::cout << "PRESSED A " << std::endl;
				// change x,y,z to safe coordinates!
				motion.x = 0;
				motion.y = 0;
				motion.z = -196;
				motion.speed = 1;
				movePathService.request.motion.push_back(motion);
				deltaRobotClient.call(movePathService);
			  	break;
			case KEYCODE_S:
				std::cout << "PRESSED S " << std::endl;
			  	break;
			case KEYCODE_D:
			  	std::cout << "PRESSED D " << std::endl;
				// change x,y,z to safe coordinates!
				motion.x = 0;
				motion.y = 0;
				motion.z = -210;
				motion.speed = 1;
				movePathService.request.motion.push_back(motion);
				deltaRobotClient.call(movePathService);
				break;
		}
		movePathService.request.motion.clear();
		ros::spinOnce();
	}
	return 0;
}