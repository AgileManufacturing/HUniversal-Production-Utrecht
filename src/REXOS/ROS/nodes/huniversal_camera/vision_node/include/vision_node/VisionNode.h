/**
 * @file CameraNode.h
 * @brief Header for the camera node class
 * @date Created: 2012-10-09
 * 
 * @author Arjan Groenewegen
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

#ifndef CAMERANODE_H
#define CAMERANODE_H

#include "ros/ros.h"
// standard services
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
// image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// camera capture
#include <camera/unicap_cv_bridge.h>
// services
#include <vision_node/autoWhiteBalance.h>
#include <vision_node/enableComponent.h>
#include <vision_node/setCorrectionMatrices.h>
#include <vision_node/getCorrectionMatrices.h>

#include <vision_node/QrCodeReader.h>
#include <vision_node/FishEyeCorrector.h>


#include <iostream>
#include <sstream>
#include <string>

class VisionNode {
public:
	VisionNode(int deviceNumber, int formatNumber, std::string pathToXmlFile);
	~VisionNode();
	void run();

	bool increaseExposure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
	bool decreaseExposure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
	bool autoWhiteBalance(vision_node::autoWhiteBalance::Request &request, vision_node::autoWhiteBalance::Response &response);
	bool enableFishEyeCorrector(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response);
	bool enableQrCodeReader(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response);
	bool enableFudicialDetector(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response);
	bool enableCamera(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response);
	
private:
	unicap_cv_bridge::UnicapCvCamera * cam;

	cv::Mat camFrame;

	ros::NodeHandle nodeHandle;

	bool isFishEyeCorrectorEnabled;
	bool isQrCodeReaderEnabled;
	bool isFudicialDetectorEnabled;
	bool isCameraEnabled;
	
	image_transport::ImageTransport imgTransport;
	image_transport::Publisher cameraFeedPublisher;
	QrCodeReader qrCodeReader;
	FishEyeCorrector fishEyeCorrector;
	

	double exposure;
};

#endif
