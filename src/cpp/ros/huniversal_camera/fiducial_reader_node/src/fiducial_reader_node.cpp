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

#include "fiducial_reader_node/fiducial_reader_node.h"
#include "fiducial_reader_node/Fiducials.h"

#include <rexos_vision/FiducialDetector.h> 

#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"

#include <iostream>

FiducialReaderNode::FiducialReaderNode() :

	it(nodeHandle),
	fiducial(8, 35)
{
	fiducialPublisher = nodeHandle.advertise<fiducial_reader_node::Fiducials>("camera/fiducials", 10);
	pub = it.advertise("camera/fiducial_debug_image", 1);
}

void FiducialReaderNode::run() {
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, &FiducialReaderNode::handleFrame, this);
	ROS_INFO("Waiting for frames");
	ros::spin();
}

void FiducialReaderNode::handleFrame(const sensor_msgs::ImageConstPtr& msg) {
	//ROS_INFO("Frame recieved");
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr2;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		cv::Mat* image = new cv::Mat(cv_ptr->image);
		cv::Mat* image2 = new cv::Mat(cv_ptr2->image);

		//Insert code here!
		fiducial_reader_node::Fiducials message;
		
		std::vector<cv::Point2f> points;
		
		fiducial.detect(*image2, points, image); //
		
		fiducial_reader_node::point64 fPoint;
		ROS_INFO_STREAM("size= " << points.size());
		for(int i = 0; i < points.size(); i++) {
			fPoint.x = points[i].x;
			fPoint.y = points[i].y;
			
			message.points.push_back(fPoint);
			//ROS_INFO_STREAM("x = "  << fPoint.x << " y = " << fPoint.y);
		//
		}
		
		ros::Rate frameRate(10);
		ros::Time time = ros::Time::now();
		
		cv_bridge::CvImage cvi;
		cvi.header.stamp = time;
		cvi.header.frame_id = "fiducial_debug_image";
		cvi.encoding = sensor_msgs::image_encodings::BGR8;
		cvi.image = *image;
		
		pub.publish(cvi.toImageMsg());
		//cv::imwrite("/home/blagtoof/Desktop/test_results/trollolololol2.png", *image);
		frameRate.sleep();
		
		delete image;
		delete image2;
		
		fiducialPublisher.publish(message);
		//ROS_INFO("image processed");
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "fucidial_reader_node");
	ROS_DEBUG("Constructing node");

	FiducialReaderNode node;

	node.run();
	return 0;
}
