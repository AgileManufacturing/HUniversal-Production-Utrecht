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

#include "qr_code_reader_node/qr_code_reader_node.h"
#include "qr_code_reader_node/qrCode.h"
#include "qr_code_reader_node/Collection.h"

#include <rexos_vision/QRCodeDetector.h> 
#include <rexos_datatypes/QrCode.h> 

#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"

#include <iostream>

QrCodeReaderNode::QrCodeReaderNode() :

	it(nodeHandle)
{
	ROS_INFO("Advertising topic");
	qrCodesPublisher = nodeHandle.advertise<qr_code_reader_node::Collection>("camera/qr_codes", 1000);
	pub = it.advertise("camera/qr_debug_image", 1);
}

void QrCodeReaderNode::run() {
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, &QrCodeReaderNode::handleFrame, this);
	ROS_INFO("Waiting for frames");
	ros::spin();
}

void QrCodeReaderNode::handleFrame(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		cv::Mat* image = new cv::Mat(cv_ptr->image);
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv::Mat* image2 = new cv::Mat(cv_ptr->image);

		//Insert code here!
		qr_code_reader_node::Collection message;
		std::vector<rexos_datatypes::QrCode> qrCodes;
		
		qr.detectQRCodes(*image, qrCodes);
		
		for(int i = 0; i < qrCodes.size(); i++){
			qr_code_reader_node::qrCode qrCode;
			qrCode.value = qrCodes[i].name;
			
			std::vector<cv::Point2f> points;
			points = qrCodes[i].getPoints();
			
			qr_code_reader_node::point64 pointA;
			pointA.x = points[0].x;
			pointA.y = points[0].y;
			qr_code_reader_node::point64 pointB;
			pointB.x = points[1].x;
			pointB.y = points[1].y;
			qr_code_reader_node::point64 pointC;
			pointC.x = points[2].x;
			pointC.y = points[2].y;
			
			qrCode.corners[0] = (pointA);
			qrCode.corners[1] = (pointB);
			qrCode.corners[2] = (pointC);
			
			message.collection.push_back(qrCode);
		
			ROS_INFO_STREAM("QR-code\tpointA = " << pointA << " pointB = " << pointB << " pointC = " << pointC << "Value = " << qrCode.value);
		}
		
		for(int i = 0; i < qrCodes.size(); i++){
			qrCodes[i].draw(*image2);
		}
		
		ros::Rate frameRate(10);
		ros::Time time = ros::Time::now();
		
		cv_bridge::CvImage cvi;
		cvi.header.stamp = time;
		cvi.header.frame_id = "qr_debug_image";
		cvi.encoding = sensor_msgs::image_encodings::BGR8;
		cvi.image = *image2;
		
		pub.publish(cvi.toImageMsg());
		//cv::imwrite("/home/t/Desktop/test_results/trollolololol.png", *image);
		frameRate.sleep();
		
		qrCodesPublisher.publish(message);
		
		delete image;
		delete image2;
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "qr_code_reader_node");
	ROS_DEBUG("Constructing node");

	QrCodeReaderNode node;

	node.run();
	return 0;
}
