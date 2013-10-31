/**
 * @file CameraNode.cpp
 * @brief Camera node
 * @date Created: 2012-10-08
 * 
 * @author Arjan Groenewegen
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

#include "vision_node/VisionNode.h"
#include "vision_node/Services.h"
#include "rexos_utilities/Utilities.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdexcept>

VisionNode::VisionNode(int deviceNumber, int formatNumber, std::string pathToXmlFile) : 
		nodeHandle(), imgTransport(nodeHandle),
		exposure(0.015), 
		isCameraEnabled(true), isFishEyeCorrectorEnabled(false),
		isQrCodeReaderEnabled(true), isFudicialDetectorEnabled(false),
		qrCodeReader(nodeHandle, imgTransport),
		fishEyeCorrector(nodeHandle)
		
{
	// Connect to camera. On failure a exception will be thrown.
	ROS_INFO("Initializing camera");
	cam = new unicap_cv_bridge::UnicapCvCamera(deviceNumber, formatNumber);
	cam->setAutoWhiteBalance(true);
	cam->setExposure(exposure);
	camFrame = cv::Mat(cam->getImgHeight(), cam->getImgWidth(), cam->getImgFormat());

	fishEyeCorrector.setFrameSize(cv::Size(cam->getImgWidth(), cam->getImgHeight()));

	ROS_INFO("Advertising services");
	// Advertise the services.
	increaseExposureService = nodeHandle.advertiseService(vision_node_services::INCREASE_EXPOSURE,
	        &VisionNode::increaseExposure, this);
	decreaseExposureService = nodeHandle.advertiseService(vision_node_services::DECREASE_EXPOSURE,
	        &VisionNode::decreaseExposure, this);
	autoWhiteBalanceService = nodeHandle.advertiseService(vision_node_services::AUTO_WHITE_BALANCE,
	        &VisionNode::autoWhiteBalance, this);
	enableCameraService = nodeHandle.advertiseService(vision_node_services::ENABLE_CAMERA,
	        &VisionNode::enableCamera, this);
	enableQrCodeReaderService = nodeHandle.advertiseService(vision_node_services::ENABLE_QR_CODE_READER,
	        &VisionNode::enableQrCodeReader, this);
	enableFishEyeCorrectorService = nodeHandle.advertiseService(vision_node_services::ENABLE_FISH_EYE_CORRECTOR,
	        &VisionNode::enableFishEyeCorrector, this);
	
	cameraFeedPublisher = imgTransport.advertise("camera/image", 1);
}

VisionNode::~VisionNode() {
	delete cam;
}

bool VisionNode::increaseExposure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
	ROS_DEBUG("Service increaseExposure ");
	exposure *= 1.125;
	ROS_DEBUG_STREAM("exposure=" << exposure);
	if(cam) {
		cam->setExposure(exposure);
		return true;
	} else {
		return false;
	}
}
bool VisionNode::decreaseExposure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
	ROS_DEBUG("Service decreaseExposure ");
	exposure /= 1.125;
	ROS_DEBUG_STREAM("exposure=" << exposure);
	if(cam) {
		cam->setExposure(exposure);
		return true;
	} else {
		return false;
	}

}
bool VisionNode::autoWhiteBalance(vision_node::autoWhiteBalance::Request& request, vision_node::autoWhiteBalance::Response& response) {
	ROS_DEBUG_STREAM("Service autoWhiteBalance " << (bool) request.enable);
	if(cam) {
		cam->setAutoWhiteBalance(request.enable);
		return true;
	} else {
		return false;
	}
}
bool VisionNode::enableFishEyeCorrector(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	ROS_DEBUG_STREAM("Service fishEyeCorrection " << request.enable);
	if(cam && fishEyeCorrector.isReady() == true) {
		isFishEyeCorrectorEnabled = request.enable;
		return true;
	} else {
		return false;
	}

}
bool VisionNode::enableCamera(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	ROS_DEBUG_STREAM("Service enableCamera " << request.enable);
	if(cam) {
		isCameraEnabled = request.enable;
		return true;
	} else {
		return false;
	}
}
bool VisionNode::enableQrCodeReader(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	ROS_DEBUG_STREAM("Service enableQrCodeReader " << request.enable);
	if(cam) {
		isQrCodeReaderEnabled = request.enable;
		return true;
	} else {
		return false;
	}
}

void VisionNode::run() {
	if(!cam) throw std::runtime_error("Camera not initialized!");

	ros::Rate frameRate(10);
	while(ros::ok()) {
		if(isCameraEnabled) {
			// Read image 
			cam->getFrame(&camFrame);
			if(isFishEyeCorrectorEnabled == true){
				camFrame = fishEyeCorrector.handleFrame(camFrame);
			}
			
			cv::Mat grayScaleFrame;
			if(isQrCodeReaderEnabled == true || isFudicialDetectorEnabled == true){
				// convert to grayscale, because these readers / detecors need grayscale images
				cvtColor(camFrame, grayScaleFrame, CV_RGB2GRAY);
			}
			
			if(isQrCodeReaderEnabled == true){
				qrCodeReader.handleFrame(grayScaleFrame, &camFrame);
			}
		
			if(cameraFeedPublisher.getNumSubscribers() != 0){
				ros::Time time = ros::Time::now();
				cv_bridge::CvImage cvi;
				cvi.header.stamp = time;
				cvi.header.frame_id = "image";
				cvi.encoding = sensor_msgs::image_encodings::BGR8;
				cvi.image = camFrame;
				cameraFeedPublisher.publish(cvi.toImageMsg());
			}
		}
		
		frameRate.sleep();
		ros::spinOnce();
	}
}
