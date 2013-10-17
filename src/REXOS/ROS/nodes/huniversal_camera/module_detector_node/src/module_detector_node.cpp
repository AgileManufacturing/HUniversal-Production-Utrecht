/**
 * @file part_locator_node.cpp
 * @brief locates objects and rotates points.
 * @date Created: 2013-09-20
 *
 * @author Garik hakopian
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
#include "ros/ros.h"

#include "module_detector_node/module_detector_node.h"

#include <iostream>

#define MODULE_QR_CODE_PREFIX "_MOD_"
#define MODULE_QR_CODE_TIME_TO_LIVE 3

ModuleDetectorNode::ModuleDetectorNode() {
	ROS_INFO("Constructing");
}

void ModuleDetectorNode::qrCodeCallback(const qr_code_reader_node::Collection & message) {
	
	int collectionSize = message.collection.size();
	int moduleQrCodePrefixLength = strlen(MODULE_QR_CODE_PREFIX);
	
	// clean up deprecated QR codes
	ros::Time currentTime = ros::Time::now();
	std::map<std::string, ros::Time>::iterator it = pendingQrCodes.begin();
	while(it != pendingQrCodes.end()) {
		ros::Duration timePast = currentTime - it->second;
		if(timePast > ros::Duration(MODULE_QR_CODE_TIME_TO_LIVE)) {
			pendingQrCodes.erase(it++);
		} else {
			it++;
		}
	}
	
	for(int i = 0; i < collectionSize; i++){
		if(message.collection[i].value.substr(0, moduleQrCodePrefixLength) == std::string(MODULE_QR_CODE_PREFIX)) {
			// we have found a QR code of a module.
			std::string data = message.collection[i].value.substr(moduleQrCodePrefixLength);
			
			// have we previously scanned this QR code?
			if(pendingQrCodes.count(data) == 0){
				ROS_INFO("Detected new module QR-code");
				std_msgs::String moduleMsg;
				moduleMsg.data = data;
				modulesPublisher.publish(moduleMsg);
				pendingQrCodes.insert(std::pair<std::string, ros::Time>(data, currentTime));
			}
		}
	}
}
	
void ModuleDetectorNode::run() {
	modulesPublisher = nodeHandle.advertise<std_msgs::String>("camera/modules", 100);
	
	ROS_INFO("waiting for camera/qr_codes");
	ros::Subscriber sub = nodeHandle.subscribe("camera/qr_codes", 10, &ModuleDetectorNode::qrCodeCallback, this);
	ros::spin();
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "module_detector_node");
	ROS_DEBUG("Constructing node");

	ModuleDetectorNode node;
	
	node.run();
	return 0;
}
