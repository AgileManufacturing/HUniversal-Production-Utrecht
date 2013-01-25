/**
 * @file CrateLocatorNode.h
 * @brief Detects the crates and publishes events on the crateEvent topic.
 * @date Created: 2011-11-11
 * @date Rerexos_visioned: 2012-10-22
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
#include <rexos_datatypes/Crate.h>
#include <rexos_datatypes/Point2D.h>
#include <rexos_vision/QRCodeDetector.h>
#include <rexos_vision/FiducialDetector.h>
#include <rexos_vision/PixelAndRealCoordinateTransformer.h>
#include <rexos_vision/CrateTracker.h>

#include <crate_locator_node/getCrate.h>
#include <crate_locator_node/getAllCrates.h>
#include <crate_locator_node/CrateEventMsg.h>


/**
 * Locates the crate
 **/
class CrateLocatorNode{
public:
	CrateLocatorNode();
	~CrateLocatorNode();

	void run();
	bool getCrate(crate_locator_node::getCrate::Request &req,crate_locator_node::getCrate::Response &res);
	bool getAllCrates(crate_locator_node::getAllCrates::Request &req,crate_locator_node::getAllCrates::Response &res);

private:
	/**
	 * @var rexos_vision::FiducialDetector * fidDetector
	 * The fiducials detector that localizes markers on the working area.
	 **/
	rexos_vision::FiducialDetector * fidDetector;

	/**
	 * @var rexos_vision::QRCodeDetector * qrDetector
	 * The QR detector that detects all QR codes within the camera frame.
	 **/
	rexos_vision::QRCodeDetector * qrDetector;

	/**
	 * @var rexos_vision::PixelAndRealCoordinateTransformer * cordTransformer
	 * Coordinate transformer that transform pixel (camera) coordinates into real life (deltarobot) coordinates.
	 * Uses three marked points relative to the robot to determine the orientation and scale.
	 **/
	rexos_vision::PixelAndRealCoordinateTransformer * cordTransformer;

	/**
	 * @var rexos_vision::CrateTracker * crateTracker
	 * The CrateTracker follows all movements of the crates. It sends events if a crate is new, moving, moved or removed.
	 **/
	rexos_vision::CrateTracker * crateTracker;

	/**
	 * @var std::vector<rexos_datatypes::Point2D> markers
	 * Vector containing the locations of the three fiducial markers.
	 **/
	std::vector<rexos_datatypes::Point2D> markers;

	/**
	 * @var ros::NodeHandle node
	 * The nodeHandle used by ros services and topics
	 **/
	ros::NodeHandle node;

	/**
	 * @var ros::Publisher crateEventPublisher
	 * Publisher for the crate event topic. The publisher opens the topic for writing.
	 **/
	ros::Publisher crateEventPublisher;

	/**
	 * @var ros::ServiceServer getCrateService
	 * Service server for the getCrateService. As long as the service server is "running" the service is provided.
	 **/
	ros::ServiceServer getCrateService;

	/**
	 * @var ros::ServiceServer getAllCratesService
	 * Service server for the getAllCratesService. As long as the service server is "running" the service is provided.
	 **/
	ros::ServiceServer getAllCratesService;

	// For calibration
	/**
	 * @var std::vector<cv::Point2f> fid1_buffer
	 * The point buffer for fiducial 1.
	 * This buffer is averaged before used as a definitive point.
	 **/
	std::vector<cv::Point2f> fid1_buffer;

	/**
	 * @var std::vector<cv::Point2f> fid2_buffer
	 * The point buffer for fiducial 12
	 * This buffer is averaged before used as a definitive point.
	 **/
	std::vector<cv::Point2f> fid2_buffer;

	/**
	 * @var std::vector<cv::Point2f> fid3_buffer
	 * The point buffer for fiducial 3.
	 * This buffer is averaged before used as a definitive point.
	 **/
	std::vector<cv::Point2f> fid3_buffer;

	/**
	 * @var unsigned int measurementCount
	 * Amount of measurements that were successful.
	 **/
	unsigned int measurementCount;

	/**
	 * @var unsigned int measurements
	 * Number of measurements to be done.
	 **/
	unsigned int measurements;

	/**
	 * @var unsigned int failCount
	 * Number of failed measurements.
	 **/
	unsigned int failCount;

	/**
	 * @var image_transport::ImageTransport imageTransport
	 * ImageTransport is used to transport camera frames over a ros topic.
	 **/
	image_transport::ImageTransport imageTransport;

	/**
	 * @var image_transport::Subscriber cameraSubscriber
	 * Subscription to the camera topic for receiving frames.
	 **/
	image_transport::Subscriber cameraSubscriber;

	bool calibrate(unsigned int measurements = 100, unsigned int maxErrors = 100);
	void calibrateCallback(const sensor_msgs::ImageConstPtr& msg);
	void crateLocateCallback(const sensor_msgs::ImageConstPtr& msg);
};
