/**
 * @file cameraCalibrationNode.cpp
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

#include "vision_node/VisionNode.h"
#include "vision_node/QrCode.h"
#include "vision_node/QrCodes.h"

#include <rexos_vision/QRCodeDetector.h> 
#include <rexos_vision/QrCode.h> 

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <iostream>

QrCodeReader::QrCodeReader(ros::NodeHandle& nodeHandle, image_transport::ImageTransport& imageTransport) {
	REXOS_INFO("Advertising topic");
	qrCodesPublisher = nodeHandle.advertise<vision_node::QrCodes>("camera/qr_codes", 10);
	debugImagePublisher = imageTransport.advertise("camera/qr_debug_image", 1);
}

void QrCodeReader::handleFrame(cv::Mat& frame, cv::Mat* debugFrame) {
	vision_node::QrCodes message;
	std::vector<rexos_datatypes::QrCode> qrCodes;
	
	qrCodeDetector.detectQRCodes(frame, qrCodes);
	
	REXOS_DEBUG_STREAM("QrCodes detected=" << qrCodes.size());
	for(int i = 0; i < qrCodes.size(); i++){
		vision_node::QrCode qrCode;
		qrCode.value = qrCodes[i].name;
		
		std::vector<cv::Point2f> points;
		points = qrCodes[i].getPoints();
		
		vision_node::Point64 pointA;
		pointA.x = points[0].x;
		pointA.y = points[0].y;
		vision_node::Point64 pointB;
		pointB.x = points[1].x;
		pointB.y = points[1].y;
		vision_node::Point64 pointC;
		pointC.x = points[2].x;
		pointC.y = points[2].y;
		
		qrCode.corners[0] = (pointA);
		qrCode.corners[1] = (pointB);
		qrCode.corners[2] = (pointC);
		
		message.qrCodes.push_back(qrCode);
	
		REXOS_INFO_STREAM("QR-code\tpointA = " << pointA << " pointB = " << pointB << " pointC = " << pointC << "Value = " << qrCode.value);
	}
	if(debugFrame != NULL && debugImagePublisher.getNumSubscribers() != 0){
		// we are changing the image, so we need a copy
		cv::Mat debugImage = cv::Mat(*debugFrame);
		
		for(int i = 0; i < qrCodes.size(); i++){
			qrCodes[i].draw(debugImage);
		}
		
		cv_bridge::CvImage cvi;
		ros::Time time = ros::Time::now();
		cvi.header.stamp = time;
		cvi.header.frame_id = "qr_debug_image";
		cvi.encoding = sensor_msgs::image_encodings::BGR8;
		cvi.image = debugImage;
		
		debugImagePublisher.publish(cvi.toImageMsg());
		//cv::imwrite("/home/t/Desktop/test_results/trollolololol.png", *image);
	}
	
	
	
	qrCodesPublisher.publish(message);
	
	}