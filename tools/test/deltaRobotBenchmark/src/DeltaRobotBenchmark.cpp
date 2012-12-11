/**
 * @file DeltaRobotBenchmark.cpp
 * @brief Test and time all the services of the DeltaRobotNode.
 * @date Created: 2012-12-11
 *
 * @author Dennis Koole
 * @author Koen Braham
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

#include "ros/ros.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/MoveRelativePath.h"
#include "deltaRobotNode/MoveToRelativePoint.h"
#include "deltaRobotNode/MoveToPoint.h"
#include "deltaRobotNode/Motion.h"
#include "deltaRobotNode/Calibrate.h"
#include "DeltaRobotNode/Services.h"

#include "Utilities/Utilities.h"

#include <sys/time.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "DeltaRobotTest"
// @endcond

namespace DeltaRobotTestNamespace {
	/**
	 * @var const double maxAcceleration
	 * The maxAcceleration in milimeters the deltarobot moves per second.
	 **/
	const double maxAcceleration = 50.0;

	/**
	 * @var char keyPress
	 * The key received from the terminal.
	 **/
	char keyPress;

	/**
	 * @var ServiceClient moveToPointClient
	 * Client to call the moveToPointService with.
	 **/
	ros::ServiceClient moveToPointClient;
	/**
	 * @var MoveToPoint moveToPointService
	 * Service to move the deltaRobot to a specific point.
	 **/
	deltaRobotNode::MoveToPoint moveToPointService;

	/**
	 * Moves the deltaRobot effector to the starting point, with all the hips horizontal.
	 **/
	void moveToStartPoint(){
		std:: cout << "Press any key to goto startpoint" << std::endl;
		std:: cin >> keyPress; 
		moveToPointService.request.motion.x = 0;
		moveToPointService.request.motion.y = 0;
		moveToPointService.request.motion.z = -196.063;	
		moveToPointService.request.motion.maxAcceleration = maxAcceleration;
		moveToPointClient.call(moveToPointService);
	}
}

using namespace DeltaRobotTestNamespace;

timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

/**
 * Starting method for the DeltaRobotTest.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * 
 * @return 0.
 **/
int main(int argc, char **argv){
	using namespace std;

	timespec time1, time2;
	Utilities::StopWatch watch("banaan1");
	int temp=1;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	watch.start();
	for (int i = 0; i< 121000000; i++)
		temp+=temp;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
	watch.stopAndPrint(stdout);
	cout<<temp;
	cout<<diff(time1,time2).tv_sec<<":"<<diff(time1,time2).tv_nsec<<endl;

	           exit(0);

	// Ros init.
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;

	// Getting Calibrate Services.
	ros::ServiceClient calibrateClient = nodeHandle.serviceClient<deltaRobotNode::Calibrate>(DeltaRobotNodeServices::CALIBRATE);
	deltaRobotNode::Calibrate calibrateService;

	moveToPointClient = nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT);

	// Getting MoveToRelativePoint Services.
	ros::ServiceClient moveToRelativePointClient = nodeHandle.serviceClient<deltaRobotNode::MoveToRelativePoint>(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT);
	deltaRobotNode::MoveToRelativePoint moveToRelativePointService;

	// Getting MovePath Service.
	ros::ServiceClient movePathClient = nodeHandle.serviceClient<deltaRobotNode::MovePath>(DeltaRobotNodeServices::MOVE_PATH);
	deltaRobotNode::MovePath movePathService;

	// Getting MoveRelativePath Service.
	ros::ServiceClient moveRelativePathClient = nodeHandle.serviceClient<deltaRobotNode::MoveRelativePath>(DeltaRobotNodeServices::MOVE_RELATIVE_PATH);
	deltaRobotNode::MoveRelativePath moveRelativePathService;
	
	// Test Calibrate Service.
	std:: cout << "Press any key to start the Calibrate" << std::endl;
	std:: cin >> keyPress;    
	calibrateClient.call(calibrateService);

	// Test MoveToPoint Service.
	std:: cout << "Press any key to start the MoveToPoint" << std::endl;
	std:: cin >> keyPress;   
	moveToPointService.request.motion.x = 10;
	moveToPointService.request.motion.y = 10;
	moveToPointService.request.motion.z = -210;	
	moveToPointService.request.motion.maxAcceleration = maxAcceleration;
	moveToPointClient.call(moveToPointService);

	moveToStartPoint();

	// MoveToPoint speed benchmark
	std::cout << "Press any key to start the MoveToPoint speed benchmark" << std::endl;
	std::cin >> keyPress;
	for(int acc = 25; acc <= 200; acc += 25){
		std::cout << "max acceleration: " << acc << std::endl;
		for(int i = 10; i <= 40; i += 10){
			moveToPointService.request.motion.x = i;
			moveToPointService.request.motion.y = i;
			moveToPointService.request.motion.z = -210;	
			moveToPointService.request.motion.maxAcceleration = acc;
			moveToPointClient.call(moveToPointService);

			moveToPointService.request.motion.x = -i;
			moveToPointService.request.motion.y = -i;
			moveToPointService.request.motion.z = -210;	
			moveToPointService.request.motion.maxAcceleration = acc;
			moveToPointClient.call(moveToPointService);		
		}
	}

	// Test MoveToRelativePoint Service.
	std:: cout << "Press any key to start the MoveToRelativePoint" << std::endl;
	std:: cin >> keyPress;   
	moveToRelativePointService.request.motion.x = -1;
	moveToRelativePointService.request.motion.y = -1;
	moveToRelativePointService.request.motion.z = -1;	
	moveToRelativePointService.request.motion.maxAcceleration = maxAcceleration;
	moveToRelativePointClient.call(moveToRelativePointService);

	moveToStartPoint();

	// Test MovePath Service.
	std:: cout << "Press any key to start the MovePathService" << std::endl;
	std:: cin >> keyPress;   
	deltaRobotNode::Motion point1;
	deltaRobotNode::Motion point2;
	deltaRobotNode::Motion point3;
	deltaRobotNode::Motion point4;
	point1.x = 10;
	point1.y = 10;
	point1.z = -210;
	point1.maxAcceleration = maxAcceleration;

	point2.x = -10;
	point2.y = -10;
	point2.z = -210;
	point2.maxAcceleration = maxAcceleration;

	point3.x = 30;
	point3.y = 30;
	point3.z = -190;
	point3.maxAcceleration = maxAcceleration;

	point4.x = -20;
	point4.y = 20;
	point4.z = -210;
	point4.maxAcceleration = maxAcceleration;
	
	movePathService.request.motion.push_back(point1);
	movePathService.request.motion.push_back(point2);
	movePathService.request.motion.push_back(point3);
	movePathService.request.motion.push_back(point4);
	movePathClient.call(movePathService);

	moveToStartPoint();

	// Test MoveRelativePath Service.
	std:: cout << "Press any key to start the MoveRelativePath" << std::endl;
	std:: cin >> keyPress; 
	for(double z = 0; z < 10; z++){
		deltaRobotNode::Motion point1;
		deltaRobotNode::Motion point2;
		deltaRobotNode::Motion point3;
		deltaRobotNode::Motion point4;
		deltaRobotNode::Motion point5;
		point1.x = 0;
		point1.y = 0;
		point1.z = -5;
		point1.maxAcceleration = maxAcceleration;

		point2.x = 20;
		point2.y = 0;
		point2.z = 0;
		point2.maxAcceleration = maxAcceleration;

		point3.x = 0;
		point3.y = 20;
		point3.z = 0;
		point3.maxAcceleration = maxAcceleration;

		point4.x = -20;
		point4.y = 0;
		point4.z = 0;
		point4.maxAcceleration = maxAcceleration;

		point5.x = 0;
		point5.y = -20;
		point5.z = 0;
		point5.maxAcceleration = maxAcceleration;
		moveRelativePathService.request.motion.push_back(point1);
		moveRelativePathService.request.motion.push_back(point2);
		moveRelativePathService.request.motion.push_back(point3);
		moveRelativePathService.request.motion.push_back(point4);
		moveRelativePathService.request.motion.push_back(point5);
	}
	moveRelativePathClient.call(moveRelativePathService);
	return 0;
}
