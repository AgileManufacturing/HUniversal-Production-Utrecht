/*
 * DeltaRobotNode.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: Dick van der Steen & Dennis Koole
 */

#include "ros/ros.h"

#define NODE_NAME "DeltaRobotNode"

int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);

	ros::spin();
	return 0;
}


