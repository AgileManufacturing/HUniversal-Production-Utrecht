/**
 * @file ImageTransformationNode.cpp
 * @brief Transforms an image passed in an argument so that it can be used by the DotMatrixPrinterNode. It sends out the image on a topic.
 * @date 2012-11-06
 *
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

#include "ImageTransformationNode/ImageTransformationNode.h"

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "ImageTransformationNode"
// @endcond

/**
 * @var WINDOW_NAME
 * Name for the opencv image window.
 **/
static const char WINDOW_NAME[] = "Image window";

/**
 * On mouse click event. 
 *
 * @param event CV_EVENT (eg. CV_EVENT_LBUTTONDOWN).
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @param flags CV_EVENT_FLAG.
 * @param param ImageTransformationNode.
 **/
void on_mouse(int event, int x, int y, int flags, void* param) {
	if (event == CV_EVENT_RBUTTONDOWN) {
		((ImageTransformationNode*)param)->publishImage();
	}
}

/**
 * Constructor.
 *
 * @param path Input image path.
 **/
ImageTransformationNode::ImageTransformationNode(std::string path) : imageTransport(nodeHandle) {

	// Advertise the services
	pub = imageTransport.advertise(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1);

	// OpenCV GUI
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
	cvSetMouseCallback(WINDOW_NAME, &on_mouse, this);

	// input image
	inputImage = cv::imread(path);

	if(inputImage.data == NULL){
		std::cerr << "Invalid input image, shutting down." << std::endl;
		exit(-1);
	}
}

/**
 * Thresholds inputImage and also resizes it if necessary.
 **/
cv::Mat ImageTransformationNode::transformImage(){
	cv::Mat sizedImage(inputImage);

	if(inputImage.cols < DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH 
		|| inputImage.rows < DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT
		|| inputImage.cols > DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH 
		|| inputImage.rows > DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT){
		//calculating scale such that the entire picture will fit, with respect to aspect ratio, on the draw field.
		double scale = std::max(
			inputImage.rows / (DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT_DOTS), 
			inputImage.cols / (DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH_DOTS));

		//size should not be at least 1 pixel in height and 1 pixel in width
		cv::Size outputSize = cv::Size(
			inputImage.cols / scale < 1 ? 1 : inputImage.cols / scale, 
			inputImage.rows / scale < 1 ? 1 : inputImage.rows / scale);

		cv::resize(inputImage, sizedImage, outputSize);
	}

	cv::Mat grayImage;
	cv::cvtColor(sizedImage, grayImage, CV_BGR2GRAY);

	return grayImage > 150;
}

/**
 * Publishes the outputImage as a CvImage with the cv_bridge.
 **/
void ImageTransformationNode::publishImage(){
	ros::Time time = ros::Time::now();
	cv_bridge::CvImage cvi;
	cvi.header.stamp = time;
	cvi.header.frame_id = "image";
	cvi.encoding = sensor_msgs::image_encodings::MONO8;
	cvi.image = outputImage;
	pub.publish(cvi.toImageMsg());
	std::cout << "Published image to " << pub.getTopic() << ". Listeners: " << pub.getNumSubscribers() << std::endl;
}

/**
 * Calls transformImage(), and starts the gui for sending with the results. 
 **/
void ImageTransformationNode::run(){
	std::cout << "Transforming.." << std::endl;
	outputImage = transformImage();

	std::cout << "Starting GUI.." << std::endl;

	cv::imshow(WINDOW_NAME, outputImage);
	while(ros::ok()){
		cv::waitKey(1);
	}
}

/**
 * Main
 *
 * @param argc Argument count.
 * @param argv Node name, inputImage path
 *
 * @return 0 if succesful, -1 if command line arguments are incorrect
 **/
int main(int argc, char** argv){
	std::cout << "Starting ros node" << std::endl;
	ros::init(argc, argv, NODE_NAME);

	if(argc != 2)
	{ 	 	
    	std::cerr << "Arguments missing or incorrect. Needed: Path to InputImage" << std::endl;
 		return -1;
  	} 

  	std::string path = argv[1];

	std::cout << "Starting imageTransformationNode" << std::endl;

	ImageTransformationNode imageTransformationNode(path);

	imageTransformationNode.run();

	return 0;
}
