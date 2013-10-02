/**
 * @file CameraControlNode.cpp
 * @brief Remote interface to adjust the camera settings in runtime.
 * @date Created: 2012-10-18
 *
 * @author Koen Braham
 * @author Daan Veltman
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
 *
 **/

#include "camera_control_node/camera_control_node.h"
#include "camera_node/Services.h"
#include <iostream>

CameraControlNode::CameraControlNode() :
		increaseExposureClient(nodeHandle.serviceClient<std_srvs::Empty>(camera_node_services::INCREASE_EXPOSURE)),
		decreaseExposureClient(nodeHandle.serviceClient<std_srvs::Empty>(camera_node_services::DECREASE_EXPOSURE)),
		autoWhiteBalanceClient(nodeHandle.serviceClient<camera_node::AutoWhiteBalance>(camera_node_services::AUTO_WHITE_BALANCE)) {
}

void CameraControlNode::increaseExposureCall() {
	std::cout << "Calling service increaseExposureClient " << printResult(increaseExposureClient.call(emptyService))
	        << std::endl;
}

void CameraControlNode::decreaseExposureCall() {
	std::cout << "Calling service decreaseExposureClient " << printResult(decreaseExposureClient.call(emptyService))
	        << std::endl;
}

void CameraControlNode::autoWhiteBalanceCall(bool enabled) {
	camera_node::AutoWhiteBalance awb;
	awb.request.enable = enabled;
	std::cout << "Calling service autoWhiteBalanceClient " << enabled << " " << printResult(autoWhiteBalanceClient.call(awb))
	        << std::endl;
}

inline std::string CameraControlNode::printResult(bool result) {
	return (result ? "[OK]" : "[FAIL]");
}

void CameraControlNode::run() {
	char key;
	while(ros::ok() && !(key == 'q' || key == 'Q')) {
		std::cin >> key;

		if(key == 'a' || key == 'A') {
			autoWhiteBalanceCall(true);
		} else if(key == 'z' || key == 'Z') {
			autoWhiteBalanceCall(false);
		} else if(key == 's' || key == 'S') {
			increaseExposureCall();
		} else if(key == 'x' || key == 'X') {
			decreaseExposureCall();
		}
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "cameraControlNode");

	CameraControlNode node;

	std::cout << "Welcome to the camera node controller. Using this tool you can adjust camera settings on the fly :)."
	        << std::endl << "A\tEnable auto white balance" << std::endl << "Z\tDisable auto white balance" << std::endl
	        << "S\tIncrease exposure" << std::endl << "X\tDecrease exposure" << std::endl << "Q\tQuit program"
	        << std::endl << "Enter a key and press the \"Enter\" button" << std::endl;

	node.run();
	return 0;
}
