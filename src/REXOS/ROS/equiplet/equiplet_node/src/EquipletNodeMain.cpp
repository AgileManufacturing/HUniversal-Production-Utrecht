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

#include <stdlib.h>

int main(int argc, char **argv) {
	std::string blackboardIP;
	bool isSimulated = false;
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--blackboard") {
			if (i + 1 < argc) {
				blackboardIP = argv[++i];
			} else {
				REXOS_ERROR("--blackboard requires one argument");
				return -1;
			}
		} else if (arg == "--isSimulated") {
			isSimulated = true;
		} else if (arg == "--isShadow") {
			isSimulated = true;
			isShadow = true;
		}
	}
	if (argc < 2) {
		REXOS_ERROR("Usage: equiplet_node (--isSimulated | --isShadow) (--blackboard <ip-address>) equipletName");
		return -2;
	}
	
	std::string equipletName = std::string(argv[argc - 1]);
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName;
	ros::init(argc, argv, nodeName);
	
	equiplet_node::EquipletNode equipletNode(equipletName, isSimulated, isShadow, blackboardIP);
	
	ros::spin();
	return 0;
}
