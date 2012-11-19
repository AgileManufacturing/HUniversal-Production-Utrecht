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

#include "ros/ros.h"
#include <EquipletNode/EquipletNode.h>
#include <Utilities/Utilities.h>

/** 
 * Main that creates the equipletNode and adds hardware modules
 **/
int main(int argc, char **argv) {

	// Check if an equiplet id is given at the command line	 
	int equipletId = 1;
	if(argc != 2 || Utilities::stringToInt(equipletId, argv[1]) != 0) {
		std::cerr << "Cannot read equiplet id from commandline. Assuming equiplet id is 1" <<std::endl;
	}
	 	
	// Set the id of the Equiplet
	std::ostringstream ss;
	ss << "Equiplet" << equipletId;
	const char* equipletName = ss.str().c_str();
	
	ros::init(argc, argv, equipletName);
	EquipletNode * equipletNode = new EquipletNode(equipletId);

	// Add some hardware modules to this equiplet
	// This should change to modules being created in the Node itself after commands on blackboard
	//Mast::HardwareModuleProperties deltaRobot(1, 1, rosMast::safe, true, true);
	//Mast::HardwareModuleProperties gripper(2, 2, rosMast::safe, true, true);
	//equipletNode->addHardwareModule(deltaRobot);
	//equipletNode->addHardwareModule(gripper);

	
	// print the hardware modules that are currently added to the Equiplet
	equipletNode->printHardwareModules();
	
	ros::Rate poll_rate(10);
	while(ros::ok()) {
		poll_rate.sleep();
		ros::spinOnce();	
	}

	// Delete the EquipletNode pointer	
	delete equipletNode;

	return 0;
}