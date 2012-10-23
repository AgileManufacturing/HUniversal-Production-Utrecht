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
#include "ros/ros.h"
#include "rosMast/StateChanged.h"
#include "rosMast/StateMachine.h"
#include <Utilities/Utilities.h>

/**
 * @var equipletNode
 * The pointer for the EquipletNode class. Needed because the stateChanged method needs
 * to call some functions from the EquipletNode object.
 **/
EquipletNode *equipletNode;

/**
 * Callback function that is called when a message is received on the equiplet_statechanged topic
 * It updates the state of a hardware module.
 * 
 * @param msg Contains the data required for a state transition
 * 
 **/
void stateChanged(const rosMast::StateChangedPtr &msg) {
	if(equipletNode->updateModuleState(msg->moduleID, rosMast::StateType(msg->state))) {
		std::cout << "The state of module " << msg->moduleID << " has been changed to " << rosMast::state_txt[msg->state] << std::endl; 
	} else{
		std::cerr << "Cannot update the state of the module " << msg->moduleID << " run for your life!" << std::endl;
	}	
}

int main(int argc, char **argv) {

	/**
	 * Check if an equiplet id is given at the command 	line
	 **/
	int equipletId = 1;
	if(argc != 2 || Utilities::str2int(equipletId, argv[1]) != 0) {
		std::cerr << "Cannot read equiplet id from commandline. Assuming equiplet id is 1" <<std::endl;
	} 

	/**
	 * Set the id of the Equiplet
	 **/
	ostringstream ss;
	ss << "Equiplet" << equipletId;
	const char* equipletName = ss.str().c_str();
	
	ros::init(argc, argv, equipletName);
	ros::NodeHandle nodeHandle;

	equipletNode = new EquipletNode(equipletId);

	/**
	 * Add some hardware modules to this equiplet
	 **/
	int moduleId = 1;
	Mast::HardwareModuleProperties deltaRobot(moduleId, 1, rosMast::safe, true, true);
	equipletNode->addHardwareModule(deltaRobot);

	/**
	 * print the hardware modules that are currently added to the Equiplet
	 **/
	equipletNode->printHardwareModules();
	
	/**
	 * Subscribe to the equipletNode_statechanged topic. On that topic messages are received that
	 * the state of a hardware module has changed.
	 **/
	ros::Subscriber sub = nodeHandle.subscribe("equiplet_statechanged", 1 , stateChanged);

	/**
	 * Publish to the requestStateChange topic. On this topic the Equiplet requests a state change by a hardware
	 * module
	 **/
	ros::Publisher publisher = nodeHandle.advertise<rosMast::StateChanged>("requestStateChange", 5);

	ros::Rate poll_rate(100);

	/**
	 * Wait until there are subscribers on the requestStateChange topic before publishing a message
	 **/
	while(publisher.getNumSubscribers() == 0) {
		poll_rate.sleep();
		ros::spinOnce();	
	}

	/**
	 * Send a message to the hardware module with id moduleId to go to the standby state
	 **/
	rosMast::StateChanged msg;	
	msg.equipletID = equipletId;
	msg.moduleID = moduleId;
	msg.state = rosMast::standby;

	publisher.publish(msg);	

	while(ros::ok()) {
		poll_rate.sleep();
		ros::spinOnce();	
	}

	/**
	 * Delete the EquipletNode pointer
	 **/
	delete equipletNode;

	return 0;
}