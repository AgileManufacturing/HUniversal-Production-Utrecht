/**
 * @file ImageTransformationNode.cpp
 * @brief Transforms the image from the camera with a GUI and adjustable adaptive threshold. It sends out the image on a topic.
 * @date 2012-11-06
 *
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

#include "ImageTransformationNode/ImageTransformationNode.h"

#define NODE_NAME "ImageTransformationNode"

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
 * @param equipletID Equiplet identifier.
 * @param moduleID Module identifier.
 **/
ImageTransformationNode::ImageTransformationNode(int equipletID, int moduleID) : imageTransport(nodeHandle) {
	blockSize = 15;
	maximum = 255;
	subtract = 15;

	// Advertise the services
	pub = imageTransport.advertise(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1);

	// OpenCV GUI
	cv::namedWindow(WINDOW_NAME, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
	cv::createTrackbar("blockSize:", WINDOW_NAME, &blockSize, maximum );
	cv::createTrackbar("subtract :", WINDOW_NAME, &subtract, maximum );

	cvSetMouseCallback(WINDOW_NAME, &on_mouse, this);
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

	// cv::Mat outputImage(25,25,CV_8U);


	// int length = outputImage.cols * outputImage.rows;
	// for(int i = 0; i < length; i++){
	// 	outputImage.data[i] = 255;	
	// }

	// for(int x = 0; x < outputImage.cols; x += 5){
	// 	for(int y = 0; y < outputImage.rows; y++){
	// 		outputImage.data[y * outputImage.rows + x] = 0;
	// 	}
	// }

	// for(int x = 0; x < outputImage.cols; x++){
	// 	for(int y = 0; y < outputImage.rows; y += 5){
	// 		outputImage.data[y * outputImage.rows + x] = 0;
	// 	}
	// }

	cvi.image = outputImage;
	pub.publish(cvi.toImageMsg());
	std::cout << "Published image to " << pub.getTopic() << ". Listeners: " << pub.getNumSubscribers() << std::endl;
}

/**
 * Transforms the image on the topic to the correct size and format and publishes to a new topic.
 *
 * @param msg The pointer to the message that contains the camera image.
 **/
void ImageTransformationNode::transformCallback() {//const sensor_msgs::ImageConstPtr& msg) {
	// Receive image
	//cv_bridge::CvImagePtr cv_ptr;
	// try {
	// 	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	// } catch (cv_bridge::Exception& e) {
	// 	ROS_ERROR("cv_bridge exception: %s", e.what());
	// 	return;
	// }

	//cv_ptr->image = cv::imread("/home/kbraham/Pictures/daniel.png");
	cv::Mat image = cv::imread("/home/arjen/Desktop/rexoslogo_no_fill.png");
	//cv_ptr->image = cv::imread("/home/arjen/Desktop/corners.png");
	if(image.data == NULL){
		std::cerr << "Invalid image" << std::endl;
		exit(1);
	}


	//calculating scale such that the entire picture will fit, with respect to aspect ratio, on the draw field.
	double scale = std::max(
		image.rows / (DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixPrinterNodeSettings::DRAW_FIELD_DOTS_PER_MM), 
		image.cols / (DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH * DotMatrixPrinterNodeSettings::DRAW_FIELD_DOTS_PER_MM));
	//size should not be at least 1 pixel in height and 1 pixel in width
	cv::Size outputSize = cv::Size(
		image.cols / scale < 1 ? 1 : image.cols / scale, 
		image.rows / scale < 1 ? 1 : image.rows / scale);

	cv::Mat resizedImage;
	cv::resize(image, resizedImage, outputSize);

	cv::Mat grayImage;
	cv::cvtColor(resizedImage, grayImage, CV_BGR2GRAY);

	outputImage = grayImage > 150;
	

	// //Threshold the image, note that blocksize has to be a multiple of 3 and >= 3.
	// cv::Mat thresholdedImage;
	// cv::adaptiveThreshold(grayImage, outputImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize < 3 ? 3 : blockSize | 1, subtract);
	//TODO determine if output is a safe image to use for sending onwards

	cv::imshow(WINDOW_NAME, outputImage);
	cv::waitKey(60000);
}

/**
 * Blocking function that contains the main loop.
 * Spins in ROS to receive frames. These will execute the callbacks.
 * This function ends when ros receives a ^c
 **/
void ImageTransformationNode::run( ) {
	//cameraSubscriber = imageTransport.subscribe("camera/image", 1, &ImageTransformationNode::transformCallback, this, image_transport::TransportHints("compressed"));

	while(ros::ok()) {
		ros::spinOnce();
	}
}

/**
 * Main
 *
 * @param argc Argument count.
 * @param argv Node name, equipletID, moduleID.
 *
 * @return 0 if succesful, -1 if command line arguments are incorrect
 **/
int main(int argc, char** argv){
	std::cout << "Starting ros node" << std::endl;
	ros::init(argc, argv, NODE_NAME);


	int equipletID = 0;
	int moduleID = 0;
	if(argc < 3 || !(Utilities::stringToInt(equipletID, argv[1]) == 0 && Utilities::stringToInt(moduleID, argv[2]) == 0))
	{ 	 	
    	std::cerr << "Cannot read equiplet id and/or moduleId from commandline please use correct values." << std::endl;
 		return -1;
  	} 

	std::cout << "Starting imageTransformationNode" << std::endl;

	ImageTransformationNode imageTransformationNode(equipletID, moduleID);

	imageTransformationNode.transformCallback();

	//imageTransformationNode.run();
	return 0;
}
