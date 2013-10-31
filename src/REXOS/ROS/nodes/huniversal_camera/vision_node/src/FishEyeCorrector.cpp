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

#include "vision_node/FishEyeCorrector.h"
#include "vision_node/Services.h"

#include <iostream>

FishEyeCorrector::FishEyeCorrector(ros::NodeHandle& nodeHandle) :
		rectifier()
{
	setCorrectionMatricesService = nodeHandle.advertiseService(vision_node_services::SET_CORRECTION_MATRICES,
		&FishEyeCorrector::setCorrectionMatrices, this);
	getCorrectionMatricesService = nodeHandle.advertiseService(vision_node_services::GET_CORRECTION_MATRICES,
		&FishEyeCorrector::getCorrectionMatrices, this);
}
void FishEyeCorrector::setFrameSize(cv::Size imageSize){
	rectifier.setImageSize(imageSize);
	isFrameSizeSet = true;
}
bool FishEyeCorrector::isReady(){
	return isFrameSizeSet && areMatricesSet;
}

cv::Mat FishEyeCorrector::handleFrame(cv::Mat& frame) {
	return rectifier.rectify(frame);
}

bool FishEyeCorrector::setCorrectionMatrices(vision_node::setCorrectionMatrices::Request& request, vision_node::setCorrectionMatrices::Response& response) {
	ROS_DEBUG("Service setCorrectionMatrices");        
	// matrices are stored in row major order
	cv::Mat distCoeffs = (cv::Mat_<double>(1,5) <<
			request.distCoeffs[0],
			request.distCoeffs[1],
			request.distCoeffs[2],
			request.distCoeffs[3],
			request.distCoeffs[4]
	);
	cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
			request.cameraMatrix.values[0],
			request.cameraMatrix.values[1],
			request.cameraMatrix.values[2],
			request.cameraMatrix.values[3],
			request.cameraMatrix.values[4],
			request.cameraMatrix.values[5],
			request.cameraMatrix.values[6],
			request.cameraMatrix.values[7],
			request.cameraMatrix.values[8]
	);

	rectifier.loadMatrices(cameraMatrix, distCoeffs);
	areMatricesSet = true;
	
	ROS_INFO_STREAM("Dist Coeffs: " << std::endl << rectifier.distCoeffs);
	ROS_INFO_STREAM("Camera matrix: " << std::endl << rectifier.cameraMatrix);
	
	return true;
}

bool FishEyeCorrector::getCorrectionMatrices(vision_node::getCorrectionMatrices::Request& request, vision_node::getCorrectionMatrices::Response& response) {
	if(areMatricesSet == true) {
		response.distCoeffs.push_back(rectifier.distCoeffs.at<double>(0));
		response.distCoeffs.push_back(rectifier.distCoeffs.at<double>(1));
		response.distCoeffs.push_back(rectifier.distCoeffs.at<double>(2));
		response.distCoeffs.push_back(rectifier.distCoeffs.at<double>(3));
		response.distCoeffs.push_back(rectifier.distCoeffs.at<double>(4));
		
		response.cameraMatrix.values[0] = rectifier.cameraMatrix.at<double>(0, 0);
		response.cameraMatrix.values[1] = rectifier.cameraMatrix.at<double>(0, 1);
		response.cameraMatrix.values[2] = rectifier.cameraMatrix.at<double>(0, 2);
		response.cameraMatrix.values[3] = rectifier.cameraMatrix.at<double>(1, 0);
		response.cameraMatrix.values[4] = rectifier.cameraMatrix.at<double>(1, 1);
		response.cameraMatrix.values[5] = rectifier.cameraMatrix.at<double>(1, 2);
		response.cameraMatrix.values[6] = rectifier.cameraMatrix.at<double>(2, 0);
		response.cameraMatrix.values[7] = rectifier.cameraMatrix.at<double>(2, 1);
		response.cameraMatrix.values[8] = rectifier.cameraMatrix.at<double>(2, 2);
		return true;
	} else {
		return false;
	}
}