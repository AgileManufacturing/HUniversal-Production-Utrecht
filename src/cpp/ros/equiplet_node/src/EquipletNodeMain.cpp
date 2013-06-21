/**
 * @file EquipletNode.cpp
 * @brief Main for EquipletNode
 * @date Created: 2012-10-12
 *
 * @author Joris Vergeer
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

#include <equiplet_node/EquipletNode.h>

const char* DEFAULT_BLACKBOARD_IP = "145.89.191.131";

static void show_usage(std::string name) {
	std::cerr << "Usage: " << name << " <options(s)>\n" << "Options:\n"
			<< "\t--help\t\tShow this help message\n"
			<< "\t--blackboard\tIP address of the blackboard (default: 145.89.191.131)\n"
			<< "\t--id\t\tID of this equiplet (default: 1)\n";
}

int main(int argc, char **argv) {
	std::string blackboardIP = DEFAULT_BLACKBOARD_IP;
	int equipletID = 1;

	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--help") {
			show_usage(argv[0]);
			return 0;
		} else if (arg == "--blackboard") {
			if (i + 1 < argc) {
				blackboardIP = argv[i++];
			} else {
				std::cerr << "--blackboard requires one argument";
				return -1;
			}
		} else if (arg == "--id") {
			if (i + 1 < argc) {
				std::stringstream ss;
				ss << argv[i++];
				ss >> equipletID;
			} else {
				std::cerr << "--id requires one argument";
				return -1;
			}
		}
	}

	// Set the id of the Equiplet
	std::string equipletName = std::string("equiplet_") + std::to_string(equipletID);

	ros::init(argc, argv, equipletName);
	equiplet_node::EquipletNode equipletNode(equipletID, blackboardIP);

	ros::Rate poll_rate(10);
	while (ros::ok()) {
		poll_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
