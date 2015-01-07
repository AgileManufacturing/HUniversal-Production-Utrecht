/**
 * @file CameraNode.cpp
 * @brief Camera node
 * @date Created: 2012-10-08
 * 
 * @author Arjan Groenewegen
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

#include "image_stream_node/image_stream_node.h"
#include "rexos_utilities/Utilities.h"

#include <boost/filesystem.hpp>

ImageStreamNode::ImageStreamNode(int argc, char * argv[]) : imageTransport(nodeHandle) {
	if(argc != 2){
		REXOS_INFO_STREAM("Usage: image_stream_node <pathToImageFile>" << std::endl);
		exit(-1);
	}
	pathToImageFile = argv[1];
	publisher = imageTransport.advertise("camera/image", 1);
	
//	boost::filesystem::is_directory(imageDir)
}

ImageStreamNode::~ImageStreamNode() {
}


void ImageStreamNode::run() {
	ros::Rate frameRate(10);
	//IplImage* img = cvLoadImage(pathToImageFile);
	cv::Mat src, dst;
	 /// Load image
	//src = cv::imread(pathToImageFile, 1 );
	/// Convert to grayscale
   /* cvtColor( src, src, CV_BGR2GRAY );
	
	
	equalizeHist( src, dst );
	
    cvtColor( dst, dst, CV_GRAY2BGR );*/
	
	
	
	
	
REXOS_INFO_STREAM("a" << std::endl);
	
	
cv::Mat img = cv::imread(pathToImageFile, CV_LOAD_IMAGE_GRAYSCALE);
REXOS_INFO_STREAM("b" << std::endl);

//create a CLAHE object (Arguments are optional).
cv::CLAHE* clahe = cv::createCLAHE(4.0, cv::Size(8, 8));
REXOS_INFO_STREAM("clahe ptr: " << (long) clahe << std::endl);
//clahe->setClipLimit(4.0);

clahe->apply(img, dst);

	
	
	
	

	/*if(img == 0){
		REXOS_ERROR("Loading image failed");
		exit(-2);
	}*//*
	while(ros::ok()) {
		ros::Time time = ros::Time::now();
		cv_bridge::CvImage cvi;
		cvi.header.stamp = time;
		cvi.header.frame_id = "image";
		cvi.encoding = sensor_msgs::image_encodings::BGR8;
		//cvi.image = img;
		cvi.image = dst;
		
		publisher.publish(cvi.toImageMsg());

		frameRate.sleep();
		ros::spinOnce();
	}*/
}

/**
 *
 * Exit codes:
 *	0 normal
 *	1 not enough arguments
 *	2 invalid device number
 *	3 invalid format number
 *	4 XML file not found
 * 	5 Unicap error
 * 	6 Running main loop without a camera
 **/
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "image_stream_node");
	
	ImageStreamNode isn(argc, argv);

	//std::cout << "[DEBUG] Advertising services" << std::endl;
	ros::NodeHandle nodeHandle;


	isn.run();

	return 0;
}
