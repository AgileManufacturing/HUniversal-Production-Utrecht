/**
 * @file CameraCalibrationNode.h
 * @brief Remote interface to adjust the camera settings in runtime.
 * @date Created: 2012-10-18
 *
 * @author Tommas Bakker
 *
 * @section LICENSE
 * Copyright © 201, HU University of Applied Sciences Utrecht.
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

#ifndef CAMERACALIBRATIONNODE_H_
#define CAMERACALIBRATIONNODE_H_

#define CAPTURE_RATE 2

#include "ros/ros.h"

#include <image_transport/image_transport.h>

#include <camera_node/camera_node.h>
#include <camera_calibration_node/calibrateLens.h>
#include <camera_calibration_node/calibrateEffector.h>

/**
 * @brief this class uses the camera/RectifyImage library to create and upload a matrix in order to rectify an image.
 * The matrix is generated using the camera feed, which provides images containing chessboard patterns
 */
class CameraCalibrationNode {
public:
	/**
	 * Constructs the node and advertices the services
	 */
	CameraCalibrationNode();
	/**
	 * This "activates" the node (allowing it to do work). This function never returns.
	 */
	void run();
	/**
	 * This is the callback function for the calibrateLens service. 
	 * This method uses the camera/RectifyImage::createMatrices() to calibrate the lens (fisheye correction).
	 */
	bool calibrateLens(camera_calibration_node::calibrateLens::Request &request, camera_calibration_node::calibrateLens::Response &response);
	/**
	 * This is the callback function for the calibrateEffector service. (not implemented)
	 */
	bool calibrateEffector(camera_calibration_node::calibrateEffector::Request &request, camera_calibration_node::calibrateEffector::Response &response);
	/**
	 * This is the callback function for the image feed. Any recieved frame is converted and saved.
	 */
	void handleFrame(const sensor_msgs::ImageConstPtr& msg);
private:
	ros::NodeHandle nodeHandle;
	ros::ServiceServer calibrateLensServer;
	ros::ServiceServer calibrateEffectorServer;
	
	/**
	 * This is the image transport object (allowing the convertion of images)
	 */
	image_transport::ImageTransport it;

	int32_t framesToCapture;
	std::vector<cv::Mat*> images;
};

#endif /* CAMERACALIBRATIONNODE_H_ */
