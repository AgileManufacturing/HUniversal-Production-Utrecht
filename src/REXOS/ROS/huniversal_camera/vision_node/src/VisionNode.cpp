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

VisionNode::VisionNode(std::string equipletName, std::vector<rexos_datatypes::ModuleIdentifier> identifier, bool isSimulated, int deviceNumber, int formatNumber) : 
		isSimulated(isSimulated), nodeHandle(),
		isCameraEnabled(true), isFishEyeCorrectorEnabled(false),
                isQrCodeReaderEnabled(true), isFudicialDetectorEnabled(false),
                isStlNodeEnabled(true),
		imgTransport(nodeHandle),
		qrCodeReader(nodeHandle, imgTransport),
                stlNode(imgTransport),
				stlNode2(imgTransport),
		fishEyeCorrector(nodeHandle),
		exposure(0.015)
{
	// Connect to camera. On failure an exception will be thrown.
	REXOS_INFO("Initializing camera");
	if(isSimulated == true) {
		QRCam = new camera::SimulatedCamera(equipletName, identifier[0], this, 5, nodeHandle);
	} else {
		auto QRCamCV = new camera::unicap_cv_bridge::UnicapCvCamera(equipletName, identifier[0], this, 5, QR_CAM_ID, formatNumber);
		QRCamCV->setAutoWhiteBalance(true);
		QRCamCV->setExposure(exposure);
		QRCam = QRCamCV;
		auto STLCamCV = new camera::unicap_cv_bridge::UnicapCvCamera(equipletName, identifier[1], this, 5, STL_CAM_ID, formatNumber);
		STLCamCV->setAutoWhiteBalance(true);
		STLCamCV->setExposure(exposure);
		STLCam = STLCamCV;
		REXOS_INFO("Initialized cameras");
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
	delete QRCam;
	delete STLCam;
}

bool VisionNode::increaseExposure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
	if(isSimulated == true) {
		REXOS_ERROR("Simulated camera does not support increaseExposure");
		return false;
	} else {
		REXOS_DEBUG("Service decreaseExposure ");
		exposure *= 1.125;
		REXOS_DEBUG_STREAM("exposure=" << exposure);
		if(QRCam && STLCam) {
			((camera::unicap_cv_bridge::UnicapCvCamera*) QRCam)->setExposure(exposure);
			((camera::unicap_cv_bridge::UnicapCvCamera*) STLCam)->setExposure(exposure);
			return true;
		} else if (QRCam){
			((camera::unicap_cv_bridge::UnicapCvCamera*) QRCam)->setExposure(exposure);
			return true;
		}
		return false;
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
		if(QRCam && STLCam) {
			((camera::unicap_cv_bridge::UnicapCvCamera*) QRCam)->setExposure(exposure);
			((camera::unicap_cv_bridge::UnicapCvCamera*) STLCam)->setExposure(exposure);
			return true;
		} else if (QRCam){
			((camera::unicap_cv_bridge::UnicapCvCamera*) QRCam)->setExposure(exposure);
			return true;
		}
		return false;
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
		if(QRCam && STLCam) {
			((camera::unicap_cv_bridge::UnicapCvCamera*) QRCam)->setAutoWhiteBalance(request.enable);
			((camera::unicap_cv_bridge::UnicapCvCamera*) STLCam)->setAutoWhiteBalance(request.enable);
			return true;
		} else if (QRCam){
			((camera::unicap_cv_bridge::UnicapCvCamera*) QRCam)->setAutoWhiteBalance(request.enable);
			return true;
		}
		return false;
	}
}
bool VisionNode::enableFishEyeCorrector(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	REXOS_DEBUG_STREAM("Service fishEyeCorrection " << request.enable);
	
	if(request.enable == true) {
		cv::Size camFramSize = QRCam->getFrameSize();
		fishEyeCorrector.setFrameSize(camFramSize);
	}
	if(QRCam && fishEyeCorrector.isReady() == true) {
		isFishEyeCorrectorEnabled = request.enable;
		return true;
	} else {
		return false;
	}
}

bool VisionNode::enableCamera(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	REXOS_DEBUG_STREAM("Service enableCamera " << request);
	if(QRCam) {
		isCameraEnabled = request.enable;
		return true;
	} else {
		return false;
	}
}
bool VisionNode::enableQrCodeReader(vision_node::enableComponent::Request& request, vision_node::enableComponent::Response& response) {
	REXOS_DEBUG_STREAM("Service enableQrCodeReader " << request);
	if(QRCam) {
		isQrCodeReaderEnabled = request.enable;
		return true;
	} else {
		return false;
	}
}

void VisionNode::run() {
	if(!QRCam) {
		REXOS_WARN_STREAM("Camera not initialized"); 
		throw std::runtime_error("Camera not initialized!");
	}
	ros::spin();
}

void VisionNode::handleFrame(cv::Mat& camFrame, int CameraID) {
	//REXOS_WARN_STREAM("\n Now handling frame from camera " << CameraID);
	if(isFishEyeCorrectorEnabled == true && CameraID == QR_CAM_ID){
		camFrame = fishEyeCorrector.handleFrame(camFrame);
		//REXOS_WARN_STREAM("Handled by FishEye");
	}
        if(isStlNodeEnabled){
			if(CameraID == STL_CAM_ID){
				stlNode.handleFrame(camFrame);
			}
			else if(CameraID == STL_CAM_ID2){
				stlNode2.handleFrame(camFrame);
			}
            //stlNode.handleFrame(camFrame);
            //REXOS_WARN_STREAM("Handled by STL");
        }
	cv::Mat grayScaleFrame;
	if((isQrCodeReaderEnabled == true || isFudicialDetectorEnabled == true)  && CameraID == QR_CAM_ID){
		// convert to grayscale, because these readers / detectors need grayscale images

		cvtColor(camFrame, grayScaleFrame, CV_RGB2GRAY);
		//REXOS_WARN_STREAM("black white succeeded");
	}
	
        if(isQrCodeReaderEnabled && CameraID == QR_CAM_ID){
		qrCodeReader.handleFrame(grayScaleFrame, &camFrame);
		//REXOS_WARN_STREAM("Handled by QR");
	}

	if(cameraFeedPublisher.getNumSubscribers() != 0){
		if (CameraID == QR_CAM_ID || (CameraID == STL_CAM_ID && isFishEyeCorrectorEnabled == true) || (CameraID == STL_CAM_ID && isFishEyeCorrectorEnabled == true)){
			//REXOS_WARN_STREAM("Subscribers is more than 0");
			ros::Time time = ros::Time::now();
			cv_bridge::CvImage cvi;	
			cvi.header.stamp = time;
			cvi.header.frame_id = "image";
			cvi.encoding = sensor_msgs::image_encodings::BGR8;
			cvi.image = camFrame;
			cameraFeedPublisher.publish(cvi.toImageMsg());
		}
		//REXOS_WARN_STREAM("Done this, been there");
	} else {
		//REXOS_WARN_STREAM("No subscribers");
	}
}
