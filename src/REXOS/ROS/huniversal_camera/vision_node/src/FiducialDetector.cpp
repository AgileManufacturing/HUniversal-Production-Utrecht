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

#include "vision_node/FiducialDetector.h"
#include "vision_node/Fiducials.h"

#include <rexos_vision/FiducialDetector.h> 

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>

#include <iostream>

FiducialDetector::FiducialDetector(ros::NodeHandle nodeHandle, image_transport::ImageTransport& imageTransport) :
	fiducialDetector(8, 35)
{
	fiducialPublisher = nodeHandle.advertise<vision_node::Fiducials>("camera/fiducials", 10);
	debugImagePublisher = imageTransport.advertise("camera/fiducial_debug_image", 1);
}

void FiducialDetector::handleFrame(cv::Mat& frame, cv::Mat* debugFrame) {
	// detect fiducials
	std::vector<cv::Point2f> points;
	
	if(debugFrame != NULL && debugImagePublisher.getNumSubscribers() != 0) {
		// we are changing the image, so we need a copy
		cv::Mat debugImage = cv::Mat(*debugFrame);
		fiducialDetector.detect(frame, points, &debugImage);
		
		cv_bridge::CvImage cvi;
		ros::Time time = ros::Time::now();
		cvi.header.stamp = time;
		cvi.header.frame_id = "fiducial_debug_image";
		cvi.encoding = sensor_msgs::image_encodings::BGR8;
		cvi.image = debugImage;
	
		debugImagePublisher.publish(cvi.toImageMsg());
		//cv::imwrite("/home/blagtoof/Desktop/test_results/trollolololol2.png", *image);
	} else {
		fiducialDetector.detect(frame, points);
	}
	
	REXOS_DEBUG_STREAM("size= " << points.size());
	
	// output message
	vision_node::Fiducials message;
	
	vision_node::Point64 fPoint;
	for(uint i = 0; i < points.size(); i++) {
		fPoint.x = points[i].x;
		fPoint.y = points[i].y;
		
		message.fudicials.push_back(fPoint);
		//REXOS_INFO_STREAM("x = "  << fPoint.x << " y = " << fPoint.y);
	//
	}
	
	fiducialPublisher.publish(message);
}
