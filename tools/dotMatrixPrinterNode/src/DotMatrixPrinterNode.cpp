/**
 * @file DotMatrixNode.cpp
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

#include <assert.h>
#include <DotMatrixPrinterNode/DotMatrixPrinterNode.h>
#include <DotMatrixPrinterNode/DotMatrixPrinterNodeSettings.h>
#include <DeltaRobotNode/Services.h>
#include <ImageTransformationNode/Topics.h>
#include <Utilities/Utilities.h>

#define NODE_NAME "DotMatrixPrinterNode"

DotMatrixPrinterNode::DotMatrixPrinterNode( ) :
		imageTransport(nodeHandle), deltaRobotClient(nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)), deltaRobotPathClient(nodeHandle.serviceClient<deltaRobotNode::MovePath>(DeltaRobotNodeServices::MOVE_PATH)), Z_LOW(0), Z_HIGH(0) {
	moveToPointService.request.motion.x = 0;
	moveToPointService.request.motion.y = 0;
	moveToPointService.request.motion.z = DotMatrixPrinterNodeSettings::DRAW_FIELD_Z_HIGH;
	moveToPointService.request.motion.maxAcceleration = DotMatrixPrinterNodeSettings::ACCELERATION;

	point.x = 0;
	point.y = 0;
	point.z = DotMatrixPrinterNodeSettings::DRAW_FIELD_Z_HIGH;
	point.maxAcceleration = DotMatrixPrinterNodeSettings::ACCELERATION;
}

DotMatrixPrinterNode::~DotMatrixPrinterNode( ) {

}

/**
 * Draws a dot at coordinate x, y.
 * @param x X coordinate of the dotted pixel
 * @param y Y coordinate of the dotted pixel
 **/
void DotMatrixPrinterNode::drawDot(double x, double y) {
	// Dotting location
	std::cout << "[DEBUG] Next dot at: " << x << " " << y << std::endl;

	// Move to X, Y, Zhigh
	moveToPointService.request.motion.x = x;
	moveToPointService.request.motion.y = y;
	moveToPointService.request.motion.z = Z_HIGH;
	deltaRobotClient.call(moveToPointService);

	if (!moveToPointService.response.succeeded) {
		std::cerr << "[ERROR] Moving to (X,Y,Z) (" << x << ", " << y << ", " << Z_HIGH << ") " << moveToPointService.response.message << std::endl;
	}

	// Move to X, Y, Zlow
	// TODO: find the Z for drawing
	moveToPointService.request.motion.x = x;
	moveToPointService.request.motion.y = y;
	moveToPointService.request.motion.z = Z_LOW;
	deltaRobotClient.call(moveToPointService);

	if (!moveToPointService.response.succeeded) {
		std::cerr << "[ERROR] Moving to (X,Y,Z) (" << x << ", " << y << ", " << Z_HIGH << ") " << moveToPointService.response.message << std::endl;
	}

	// Move to X, Y, Zhigh
	moveToPointService.request.motion.x = x;
	moveToPointService.request.motion.y = y;
	moveToPointService.request.motion.z = Z_HIGH;
	deltaRobotClient.call(moveToPointService);

	if (!moveToPointService.response.succeeded) {
		std::cerr << "[ERROR] Moving to (X,Y,Z) (" << x << ", " << y << ", " << Z_HIGH << ") " << moveToPointService.response.message << std::endl;
	}
}

/**
 * Draws a dot at coordinate x, y.
 * @param x X coordinate of the dotted pixel
 * @param y Y coordinate of the dotted pixel
 **/
void DotMatrixPrinterNode::drawDotToPath(double x, double y) {
	point.x = x;
	point.y = y;
	movePathService.request.motion.push_back(point);
	point.z = Z_LOW;
	movePathService.request.motion.push_back(point);
	point.z = Z_HIGH;
	movePathService.request.motion.push_back(point);
}

/**
 * Transforms the image on the topic to the correct size and format and publishes to a new topic.
 *
 * @param msg The pointer to the message that contains the camera image.
 **/
void DotMatrixPrinterNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	// Receive image
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	std::cout << "Encoding: " << cv_ptr->encoding << std::endl;

	int cols = cv_ptr->image.cols;
	int rows = cv_ptr->image.rows;

	assert(cols <= DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH * DotMatrixPrinterNodeSettings::DRAW_FIELD_DOTS_PER_MM);
	assert(rows <= DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixPrinterNodeSettings::DRAW_FIELD_DOTS_PER_MM);

	// Calculate X,Y by converting X,Y in pixels to X,Y in mm
	// ===========
	// =+--------=  + = Startpoint of the drawing (0,0)
	// =--o------=	x = endpoint of the drawing
	// =----x----=  x = Startpoint of deltarobot (0,0)
	// =---------=	o = Offset starting point of the deltaRobot (drawX, drawY)
	// =---------=
	// ===========
	double offsetX = -(cols * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT / 2);
	double offsetY = rows  * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT / 2;

	// Clear the old path.
	movePathService.request.motion.clear();
	point.z = Z_HIGH;

	double drawX, drawY;

	// moves left to right, right to left
	for(int row = 0; row < rows; row++){
		drawY = (-row + offsetY) * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT;
		if(row % 2 == 0){
			for(int col = 0; col < cols; col++){
				if (cv_ptr->image.data[row * cols + col] == 0) {
					drawX = (col + offsetX) * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT;
					drawDotToPath(drawX, drawY);
				}
			}
		} else {
			for(int col = cols - 1; col >= 0; col--){
				if (cv_ptr->image.data[row * cols + col] == 0) {
					drawX = (col + offsetX) * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT;
					drawDotToPath(drawX, drawY);
				}
			}
		}
	}

	Utilities::StopWatch stopwatch("PathTimer", true);
	movePathService.response.message = "";
	deltaRobotPathClient.call(movePathService); //path drawing
	if (movePathService.response.message.compare("") != 0) {
		std::cout << "[ERROR]" << movePathService.response.message << std::endl;
	}
	stopwatch.stopAndPrint(stdout);

	// Move back to its starting point.
	moveToPointService.request.motion.x = 0;
	moveToPointService.request.motion.y = 0;
	moveToPointService.request.motion.z = -196.063;
	deltaRobotClient.call(moveToPointService);
}

void DotMatrixPrinterNode::calibratePencilHeight(){

	point.maxAcceleration = 2.5;
	
	bool lowCalibrated = false;

	while(ros::ok() && !lowCalibrated){
		std::string input;

		std::cout << "Calibrating pencil Z-axis. Enter desired coordinate:" << std::endl;
		std::cin >> input;

		
		Z_LOW = Utilities::stringToDouble(input);
		Z_HIGH = Z_LOW + 10;


		while(ros::ok()){
			std::cout << "Trying " << Z_LOW << std::endl;

			movePathService.request.motion.clear();
			point.z = Z_HIGH;
			drawDotToPath(0,0);
			deltaRobotPathClient.call(movePathService);

			if (movePathService.response.message.compare("") != 0) {
				std::cout << "[ERROR]" << movePathService.response.message << std::endl;
				break;
			}

			char correct;

			std::cout << "Enter R to repeat current, Y to accept, or anything else to try a new coordinate." << std::endl;
			std::cin >> correct;

			if(correct == 'R' || correct == 'r'){
				continue;
			} else if(correct == 'Y' || correct == 'y'){
				Z_HIGH = Z_LOW + 10;
				lowCalibrated = true;
			}
			break;
		}
	}

	bool highCalibrated = false;

	while(ros::ok() && !highCalibrated){
		std::string offset = "";
		std::cout << "Testing top height. Enter desired offset." << std::endl;
		std::cin >> offset;

		Z_HIGH = Z_LOW + Utilities::stringToDouble(offset);
		
		while(ros::ok()){
			std::cout << "Trying " << Z_HIGH << std::endl;
			
			movePathService.request.motion.clear();
			
			point.z = Z_HIGH;
			
			drawDotToPath(DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2);
			drawDotToPath(-DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, -DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2);
			
			deltaRobotPathClient.call(movePathService);
			
			if (movePathService.response.message.compare("") != 0) {
				std::cout << "[ERROR]" << movePathService.response.message << std::endl;
				break;
			}

			char correct;

			std::cout << "Enter R to repeat current, Y to accept, or anything else to try a new offset." << std::endl;
			std::cin >> correct;

			if(correct == 'R' || correct == 'r'){
				continue;
			} else if(correct == 'Y' || correct == 'y'){
				highCalibrated = true;
			}
			break;
		}
	}
	point.maxAcceleration = DotMatrixPrinterNodeSettings::ACCELERATION;
}

/**
 * Blocking function that contains the main loop.
 * Spins in ROS to receive frames. These will execute the callbacks.
 * This function ends when ros receives a ^c
 **/
void DotMatrixPrinterNode::run( ) {

	calibratePencilHeight();

	// Z_LOW = -196.063;

	// // TODO remove hacks!
	// Z_LOW = -265.25;
	// Z_HIGH = Z_LOW + 3;

	// Move back to its starting point.
	moveToPointService.request.motion.x = 0;
	moveToPointService.request.motion.y = 0;
	moveToPointService.request.motion.z = -196.063;
	deltaRobotClient.call(moveToPointService);

	imageSubscriber = imageTransport.subscribe(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1, &DotMatrixPrinterNode::imageCallback, this);//, image_transport::TransportHints("compressed"));
	
		std::cout << "DotMatrixPrinter started. Waiting for image on " << ImageTransformationNodeTopics::TRANSFORMED_IMAGE << std::endl;

	ros::Rate loopRate(1);

	while (ros::ok()) {
		ros::spinOnce();
		loopRate.sleep();
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, NODE_NAME);

	DotMatrixPrinterNode dotMatrixNode;
	dotMatrixNode.run();

	return 0;
}

