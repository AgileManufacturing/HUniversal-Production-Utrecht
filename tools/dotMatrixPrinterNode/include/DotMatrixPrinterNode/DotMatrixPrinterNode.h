/**
 * @file DotMatrixPrinterNode.h
 * @brief A node that can use the deltarobot to print black/white images somewhat similar to a dot matrix printer.
 * @date Created: 2012-11-06
 *
 * @author Koen Braham
 * @author Daan Veltman
 * @author Arjen van Zanten
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

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <DataTypes/Point3D.h>
#include <deltaRobotNode/MoveToPoint.h>
#include <deltaRobotNode/MovePath.h>
#include <DotMatrixPrinterNode/DotMatrixPrinterNodeSettings.h>

/**
 * A node that can use the deltarobot to print black/white images somewhat similar to a dot matrix printer.
 **/
class DotMatrixPrinterNode {
public:
	DotMatrixPrinterNode();
	virtual ~DotMatrixPrinterNode();
	void run();
private:
	/**
	 * @var ros::NodeHandle nodeHandle
	 * The nodeHandle used by ros services and topics
	 **/
	ros::NodeHandle nodeHandle;

	/**
	 * @var image_transport::ImageTransport imageTransport
	 * ImageTransport is used to read images from the topic.
	 **/
	image_transport::ImageTransport imageTransport;

	/**
	 * @var image_transport::Subscriber imageSubscriber
	 * Subscription to the image topic for receiving frames.
	 **/
	image_transport::Subscriber imageSubscriber;

	/**
	 * @var ros::ServiceClient deltaRobotClient
	 * Ros service client for calling the deltarobot's moveToPoint service.
	 **/
	ros::ServiceClient deltaRobotClient;

	/**
	 * @var ros::Serviceclient deltaRobotPathClient
	 * Ros service client for calling the deltarobot's movePath service. 
	 **/
	ros::ServiceClient deltaRobotPathClient;

	/**
	 * @var deltaRobotNode::MovePath movePathService
	 * Service to move the deltaRobot along a path.
	 **/
	deltaRobotNode::MovePath movePathService;

	/**
	 * @var Point3D effectorLocation
	 * 3D Point of the deltarobot effector location. used in various movements.
	 **/
	DataTypes::Point3D<double> effectorLocation;

	/**
	 * @var double zDrawingSurface
	 * The Z coordinate of the drawing surface.
	 **/
	double zDrawingSurface;

	void deltaRobotMoveToPoint(double x, double y, double z, double maxAcceleration = DotMatrixPrinterNodeSettings::ACCELERATION);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void calibrateDrawingHeight();
	void drawDotToPath(double x, double y);
	void moveToStartPoint();
	void deltaRobotStartPath();
};
