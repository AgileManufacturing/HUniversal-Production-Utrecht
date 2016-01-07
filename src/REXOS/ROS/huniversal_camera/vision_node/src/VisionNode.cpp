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

#include <vision_node/VisionNode.h>
#include <vision_node/Services.h>
#include <camera/unicap_cv_bridge.h>
#include <camera/SimulatedCamera.h>
#include "rexos_utilities/Utilities.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdexcept>

VisionNode::VisionNode(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, int deviceNumber, int formatNumber) : 
		isSimulated(isSimulated), nodeHandle(),
		isCameraEnabled(true), isFishEyeCorrectorEnabled(false),
                isQrCodeReaderEnabled(false), isFudicialDetectorEnabled(false),
                isStlNodeEnabled(true),
		imgTransport(nodeHandle),
		qrCodeReader(nodeHandle, imgTransport),
		fishEyeCorrector(nodeHandle),
		exposure(0.015)
{
	// Connect to camera. On failure an exception will be thrown.
	REXOS_INFO("Initializing camera");
	if(isSimulated == true) {
		cam = new camera::SimulatedCamera(equipletName, identifier, this, 5, nodeHandle);
	} else {
		auto cvCam = new camera::unicap_cv_bridge::UnicapCvCamera(equipletName, identifier, this, 5, deviceNumber, formatNumber);
		cvCam->setAutoWhiteBalance(true);
		cvCam->setExposure(exposure);
		cam = cvCam;
	}
	
	REXOS_INFO("Advertising services");
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
	if(isSimulated == true) {
		REXOS_ERROR("Simulated camera does not support increaseExposure");
		return false;
	} else {
		REXOS_DEBUG("Service decreaseExposure ");
		exposure *= 1.125;
		REXOS_DEBUG_STREAM("exposure=" << exposure);
		if(cam) {
			((camera::unicap_cv_bridge::UnicapCvCamera*) cam)->setExposure(exposure);
			return true;
		} else {
			return false;
		}
	}
}
bool VisionNode::decreaseExposure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
	if(isSimulated == true) {
		REXOS_ERROR("Simulated camera does not support decreaseExposure");
		return false;
	} else {
		REXOS_DEBUG("Service decreaseExposure ");
		exposure /= 1.125;
		REXOS_DEBUG_STREAM("exposure=" << exposure);
		if(cam) {
			((camera::unicap_cv_bridge::UnicapCvCamera*) cam)->setExposure(exposure);
			return true;
		} else {
			return false;
		}
	}
}
bool VisionNode::autoWhiteBalance(vision_node::autoWhiteBalance::Request& request, vision_node::autoWhiteBalance::Response& response) {
	if(isSimulated == true) {
		REXOS_ERROR("Simulated camera does not support autoWhiteBalance");
		return false;
	} else {
		REXOS_DEBUG("Service decreaseExposure ");
		exposure *= 1.125;
		REXOS_DEBUG_STREAM("exposure=" << exposure);
		if(cam) {
			((camera::unicap_cv_bridge::UnicapCvCamera*) cam)->setAutoWhiteBalance(request.enable);
			return true;
		} else {
			return false;
		}
	}
}
bool VisionNode::enableFishEyeCorrector(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	REXOS_DEBUG_STREAM("Service fishEyeCorrection " << request.enable);
	
	if(request.enable == true) {
		cv::Size camFramSize = cam->getFrameSize();
		fishEyeCorrector.setFrameSize(camFramSize);
	}
	if(cam && fishEyeCorrector.isReady() == true) {
		isFishEyeCorrectorEnabled = request.enable;
		return true;
	} else {
		return false;
	}

}
bool VisionNode::enableCamera(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	REXOS_DEBUG_STREAM("Service enableCamera " << request);
	if(cam) {
		isCameraEnabled = request.enable;
		return true;
	} else {
		return false;
	}
}
bool VisionNode::enableQrCodeReader(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	REXOS_DEBUG_STREAM("Service enableQrCodeReader " << request);
	if(cam) {
		isQrCodeReaderEnabled = request.enable;
		return true;
	} else {
		return false;
	}
}

void VisionNode::run() {
	if(!cam) throw std::runtime_error("Camera not initialized!");
	ros::spin();
}

void VisionNode::handleFrame(cv::Mat& camFrame) {
	if(isFishEyeCorrectorEnabled == true){
		camFrame = fishEyeCorrector.handleFrame(camFrame);
	}
        if(isStlNodeEnabled){
            stlNode.handleFrame(camFrame);
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
