/**
* @file Main.cpp
* @brief Symbolizes an entire Equiplet.
* @date Created: 2012-10-09
*
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

#include <EquipletNode/EquipletNode.h>
#include <sstream>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosMast/StateChanged.h"
#include "rosMast/StateMachine.h"

#define TOP_CAMERA "TopCamera"
#define BOTTOM_CAMERA "BottomCamera"
#define DELTAROBOT "DeltaRobot1"
#define GRIPPER2 "Gripper2"

EquipletNode equiplet;

void stateChanged(const rosMast::StateChangedPtr &msg) {
	int moduleType = msg->moduleID;
	rosMast::StateType state = rosMast::StateType(msg->state);
	equiplet->updateModuleState(moduleType, stateChanged);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "Equiplet1");
	equiplet = new EquipletNode("Equiplet1");
	ros::NodeHandle nh;
	
	equiplet.updateOperationState();
	ros::Subscriber sub = nh.subscribe("equiplet_statechanged", 5 , stateChanged);
	//ros::Publisher stateRequestPublisher = nodeHandle.advertise<std_msgs::String>("chatter", 1000);

	//ros::Rate loop_rate(10); 

	// while(ros::ok()) {
	//     std_msgs::String msg;

	//     std::stringstream ss;
	//     ss << "Seconds since epoch: " << time(NULL);
	//     msg.data = ss.str();

	//     stateRequestPublisher.publish(msg);

	//     ros::spinOnce();
	//     loop_rate.sleep();
	// }


	// Mast::HardwareModuleProperties deltarobot(1, 1, Mast::start, true, false);
	// equiplet.addHardwareModule(deltarobot);
	// std::cout << "Added hardware module!" << std::endl;
	// char key;
	// while(true){
	// 	std::cin >> key;
	// 	std::cout << "key pressed!" << std::endl;
	// }
	// ros::spin();


	//equiplet.readFromBlackboard();

	// Mast::HardwareModuleProperties topCamera(TOP_CAMERA, Mast::safe, false, false);
	// Mast::HardwareModuleProperties bottomCamera(BOTTOM_CAMERA, Mast::safe, false, false);
	// Mast::HardwareModuleProperties deltarobot(DELTAROBOT, Mast::start, true, false);
	// Mast::HardwareModuleProperties gripper2(GRIPPER2, Mast::standby, true, true);

	// // Add the three hardware modules
	// std::cout << "Add four hardware modules..." << std::endl;
	// equiplet.addHardwareModule(topCamera);
	// equiplet.addHardwareModule(bottomCamera);
	// equiplet.addHardwareModule(deltarobot);
	// equiplet.addHardwareModule(gripper2);

	// equiplet.printHardwareModules();

	// std::cout << std::endl;
	// //std::cout << "safety state: " << safetyState << std::endl;
	// //std::cout << "operation state: " << operationState << std::endl; 

	// std::cout << std::endl;
	// std::cout << "Remove the hardware module " << GRIPPER2 << std::endl;
	// equiplet.removeHardwareModule(GRIPPER2);

	// equiplet.printHardwareModules();

	// std::cout << std::endl;
	// //std::cout << "safety state: " << safetyState << std::endl;
	// //std::cout << "operation state: " << operationState << std::endl; 

	delete equiplet;

	return 0;
}