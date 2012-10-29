/**
 * @file CrateLocatorNode.h
 * @brief Detects the crates and publishes events on the crateEvent topic.
 * @date Created: 2011-11-11
 * @date Revisioned: 2012-10-22
 *
 * @author Kasper van Nieuwland
 * @author Zep Mouris
 * @author Koen Braham
 * @author Daan Veltman
 *
 * @section LICENSE
 * License: newBSD
 *
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
 **/


#pragma once

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

#include <DataTypes/Point2D.h>
#include <Vision/QRCodeDetector.h>
#include <Vision/FiducialDetector.h>
#include <Vision/PixelToRealCoordinateTransformer.h>
#include <Vision/CrateTracker.h>

#include <crateLocatorNode/getCrate.h>
#include <crateLocatorNode/getAllCrates.h>
#include <crateLocatorNode/CrateEventMsg.h>

/*#include <vision/CrateEventMsg.h>
#include <vision/error.h>
#include <vision/registerNode.h>
#include <vision/getCrate.h>
#include <vision/getAllCrates.h>
*/#include <std_srvs/Empty.h>

class CrateLocatorNode{
public:

	/**
	 * the constructor
	 * @param argc use the parameters form main
	 * @param argv use the parameters form main
	 */
	CrateLocatorNode();
	/**
	 * the destructor
	 */
	~CrateLocatorNode();

	/**
	 * blocking function that contains the main loop: take frame, detect crates, send event. this function ends when ros receives a ^c
	 */
	void run();

	/**
	 * callback function for the getCrate services of ROS
	 * @param req the request object
	 * @param res the response object
	 * @return
	 */
	bool getCrate(crateLocatorNode::getCrate::Request &req,crateLocatorNode::getCrate::Response &res);
	/**
	 * callback function for the getAllCrates services of ROS
	 * @param req the request object
	 * @param res the response object
	 * @return
	 */
	bool getAllCrates(crateLocatorNode::getAllCrates::Request &req,crateLocatorNode::getAllCrates::Response &res);

private:
	Vision::FiducialDetector * fidDetector;
	Vision::QRCodeDetector * qrDetector;
	Vision::PixelToRealCoordinateTransformer * cordTransformer;
	Vision::CrateTracker * crateTracker;
	std::vector<DataTypes::Point2D> markers;

	ros::NodeHandle node;
	ros::Publisher crateEventPublisher;
	ros::Publisher ErrorPublisher;
	ros::Publisher nodeRegistrationPublisher;
	ros::ServiceServer getCrateService;
	ros::ServiceServer getAllCratesService;

	double crateMovementThresshold;
	int numberOfStableFrames;

	// For calibration
	std::vector<cv::Point2f> fid1_buffer;
	std::vector<cv::Point2f> fid2_buffer;
	std::vector<cv::Point2f> fid3_buffer;
	unsigned int measurementCount;
	unsigned int measurements;
	unsigned int failCount;

	image_transport::ImageTransport imageTransport;
	image_transport::Subscriber sub;

	bool calibrate(unsigned int measurements = 100, unsigned int maxErrors = 100);
	void calibrateCallback(const sensor_msgs::ImageConstPtr& msg);
	void crateLocateCallback(const sensor_msgs::ImageConstPtr& msg);
};
