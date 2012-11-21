/**
 * @file DotMatrixNode.h
 * @brief Semi dot matrix printer for grayscale image files.
 * @date Created: 2012-11-06
 *
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

#include <iostream>
#include <sstream>
#include <fstream>

#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <DataTypes/Point3D.h>

#include <deltaRobotNode/MoveToPoint.h>
#include <deltaRobotNode/MovePath.h>

#include <DotMatrixPrinterNode/DotMatrixPrinterNodeSettings.h>

class DotMatrixPrinterNode {
public:

	DotMatrixPrinterNode( );
	virtual ~DotMatrixPrinterNode( );

	void run( );
private:

	/**
	 * @var ros::NodeHandle node
	 * The nodeHandle used by ros services and topics
	 **/
	ros::NodeHandle nodeHandle;

	/**
	 * @var image_transport::ImageTransport imageTransport
	 * ImageTransport is used to transport camera frames over a ros topic.
	 **/
	image_transport::ImageTransport imageTransport;

	/**
	 * @var image_transport::Subscriber cameraSubscriber
	 * Subscription to the camera topic for receiving frames.
	 **/
	image_transport::Subscriber imageSubscriber;

	ros::ServiceClient deltaRobotClient;
	ros::ServiceClient deltaRobotPathClient;

	/**
	 * @var deltaRobotNode::MovePath movePathService
	 * Service to move the deltaRobot along a path.
	 **/
	deltaRobotNode::MovePath movePathService;

	/**
	 * @var deltaRobotNode::Motion effectorLocation
	 * Point for the movePathService.
	 **/
	DataTypes::Point3D<double> effectorLocation;

	double zDrawingSurface;

	void moveToStartPoint();
	void deltaRobotMoveToPoint(double x, double y, double z, double maxAcceleration = DotMatrixPrinterNodeSettings::ACCELERATION);
	void calibrateDrawingHeight();
	void drawDotToPath(double x, double y);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};
