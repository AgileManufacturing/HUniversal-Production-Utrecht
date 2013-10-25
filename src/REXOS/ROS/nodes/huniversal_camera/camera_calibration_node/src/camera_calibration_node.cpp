/**
 * @file cameraCalibrationNode.cpp
 * @brief Remote interface to adjust the camera settings in runtime.
 * @date Created: 2012-10-18
 *
 * @author Tommas Bakker
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

#include "camera_calibration_node/camera_calibration_node.h"
#include "camera_calibration_node/Services.h"

#include "vision_node/Services.h"
#include "vision_node/setCorrectionMatrices.h"

#include <camera/RectifyImage.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"

#include <iostream>

CameraCalibrationNode::CameraCalibrationNode() :

	imageTransport(nodeHandle)
{
	calibrateLensServer = 		nodeHandle.advertiseService(camera_calibration_node_services::CALIBRATE_LENS, 		&CameraCalibrationNode::calibrateLens, this);
	calibrateEffectorServer = 	nodeHandle.advertiseService(camera_calibration_node_services::CALIBRATE_EFFECTOR, 	&CameraCalibrationNode::calibrateEffector, this);
}

void CameraCalibrationNode::run() {
	ROS_DEBUG("Waiting for service calls");
	ros::spin();
}

bool CameraCalibrationNode::calibrateLens(
	camera_calibration_node::calibrateLens::Request &request,
	camera_calibration_node::calibrateLens::Response &response)
{
	int32_t framesToCapture = request.frameCount;
	// subscribe to the camera/image feed. The buffersize of 1 is intentional.
	image_transport::Subscriber imageSubscriber = imageTransport.subscribe("camera/image", 1, &CameraCalibrationNode::handleFrame, this);

	// capture required frames
	ros::Rate captureRate(CAPTURE_RATE);
	while(images.size() < framesToCapture){
		captureRate.sleep();
		ros::spinOnce();
	}
	imageSubscriber.shutdown();
	
	// createMatrices
	ROS_DEBUG("Generating matrices...");
	Camera::RectifyImage rectifier;
	int successes = rectifier.createMatrices(cv::Size(request.boardWidth, request.boardHeight), images);
	
	// did we detect a chessboard on any of the captured frames?
	if(successes != 0){
		ROS_DEBUG("Sending matrices...");
		ROS_INFO_STREAM("Dist Coeffs:" << std::endl << rectifier.distCoeffs);
		ROS_INFO_STREAM("Camera matrix:" << std::endl << rectifier.cameraMatrix);

		ros::ServiceClient client = nodeHandle.serviceClient<vision_node::setCorrectionMatrices>(vision_node_services::SET_CORRECTION_MATRICES);
		vision_node::setCorrectionMatrices serviceCall;
		
		// yeah, this is quite ugly
		serviceCall.request.distCoeffs.push_back(rectifier.distCoeffs.at<double>(0));
		serviceCall.request.distCoeffs.push_back(rectifier.distCoeffs.at<double>(1));
		serviceCall.request.distCoeffs.push_back(rectifier.distCoeffs.at<double>(2));
		serviceCall.request.distCoeffs.push_back(rectifier.distCoeffs.at<double>(3));
		serviceCall.request.distCoeffs.push_back(rectifier.distCoeffs.at<double>(4));

		serviceCall.request.cameraMatrix.values[0] = rectifier.cameraMatrix.at<double>(0, 0);
		serviceCall.request.cameraMatrix.values[1] = rectifier.cameraMatrix.at<double>(0, 1);
		serviceCall.request.cameraMatrix.values[2] = rectifier.cameraMatrix.at<double>(0, 2);
		serviceCall.request.cameraMatrix.values[3] = rectifier.cameraMatrix.at<double>(1, 0);
		serviceCall.request.cameraMatrix.values[4] = rectifier.cameraMatrix.at<double>(1, 1);
		serviceCall.request.cameraMatrix.values[5] = rectifier.cameraMatrix.at<double>(1, 2);
		serviceCall.request.cameraMatrix.values[6] = rectifier.cameraMatrix.at<double>(2, 0);
		serviceCall.request.cameraMatrix.values[7] = rectifier.cameraMatrix.at<double>(2, 1);
		serviceCall.request.cameraMatrix.values[8] = rectifier.cameraMatrix.at<double>(2, 2);
		
		client.call(serviceCall);
	}

	ROS_DEBUG("Cleaning up...");
	while(images.size() != 0){
		cv::Mat* image = images.back();
		images.pop_back();
		delete image;
	}
	
	response.processedFrames = successes;
	ROS_DEBUG("Done");
	return true;
}
bool CameraCalibrationNode::calibrateEffector(
		camera_calibration_node::calibrateEffector::Request &request, 
		camera_calibration_node::calibrateEffector::Response &response)
{
	return true;
}

void CameraCalibrationNode::handleFrame(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	cv::Mat* image = new cv::Mat(cv_ptr->image);
	
	images.push_back(image);
	//cv::imwrite("/home/agileman/Desktop/images/image.jpg", *image);
	ROS_DEBUG("image captured");
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "camera_calibration_node");

	CameraCalibrationNode node;

	node.run();
	return 0;
}
