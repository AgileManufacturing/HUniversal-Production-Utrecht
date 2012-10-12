/**
 * @file DeltaRobotTest.cpp
 * @brief Test all the services of the DeltaRobotNode.
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
 */

 #include "ros/ros.h"
 #include "deltaRobotNode/MovePath.h"
 #include "deltaRobotNode/MoveRelativePath.h"
 #include "deltaRobotNode/MoveToRelativePoint.h"
 #include "deltaRobotNode/MoveToPoint.h"
 #include "deltaRobotNode/Motion.h"
 #include "deltaRobotNode/Calibrate.h"
 #include "DeltaRobotNode/Services.h"

 #define NODE_NAME "DeltaRobotTest"

 const double speed = 100.0;
 char keyPress;

 // Getting MoveToPoint Services.
 ros::ServiceClient moveToPointClient;
 deltaRobotNode::MoveToPoint moveToPointService;

 void moveToStartPoint(){
	std:: cout << "Press any key to goto startpoint" << std::endl;
    std:: cin >> keyPress; 
	moveToPointService.request.motion.x = 0;
	moveToPointService.request.motion.y = 0;
	moveToPointService.request.motion.z = -196.063;	
	moveToPointService.request.motion.speed = speed;
	moveToPointClient.call(moveToPointService);
 }

 int main(int argc, char **argv){
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
	moveToPointService.request.motion.speed = speed;
	moveToPointClient.call(moveToPointService);

	moveToStartPoint();

	// Test MoveToRelativePoint Service.
	std:: cout << "Press any key to start the MoveToRelativePoint" << std::endl;
    std:: cin >> keyPress;   
	moveToRelativePointService.request.motion.x = -1;
	moveToRelativePointService.request.motion.y = -1;
	moveToRelativePointService.request.motion.z = -1;	
	moveToRelativePointService.request.motion.speed = speed;
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
	point1.speed = speed;

	point2.x = -10;
	point2.y = -10;
	point2.z = -210;
	point2.speed = speed;

	point3.x = 30;
	point3.y = 30;
	point3.z = -190;
	point3.speed = speed;

	point4.x = -20;
	point4.y = 20;
	point4.z = -210;
	point4.speed = speed;
	
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
		point1.speed = speed;

		point2.x = 20;
		point2.y = 0;
		point2.z = 0;
		point2.speed = speed;

		point3.x = 0;
		point3.y = 20;
		point3.z = 0;
		point3.speed = speed;

		point4.x = -20;
		point4.y = 0;
		point4.z = 0;
		point4.speed = speed;

		point5.x = 0;
		point5.y = -20;
		point5.z = 0;
		point5.speed = speed;
		moveRelativePathService.request.motion.push_back(point1);
		moveRelativePathService.request.motion.push_back(point2);
		moveRelativePathService.request.motion.push_back(point3);
		moveRelativePathService.request.motion.push_back(point4);
		moveRelativePathService.request.motion.push_back(point5);
	}
	moveRelativePathClient.call(moveRelativePathService);
 	return 0;
 }