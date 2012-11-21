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

#include <DotMatrixPrinterNode/DotMatrixPrinterNode.h>
#include <DeltaRobotNode/Services.h>
#include <ImageTransformationNode/Topics.h>
#include <Utilities/Utilities.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "DotMatrixPrinterNode"
// @endcond

DotMatrixPrinterNode::DotMatrixPrinterNode( ) :
	imageTransport(nodeHandle), 
	deltaRobotClient(nodeHandle.serviceClient<deltaRobotNode::MoveToPoint>(DeltaRobotNodeServices::MOVE_TO_POINT)), 
	deltaRobotPathClient(nodeHandle.serviceClient<deltaRobotNode::MovePath>(DeltaRobotNodeServices::MOVE_PATH)), 
	zDrawingSurface(0)
{ }

DotMatrixPrinterNode::~DotMatrixPrinterNode( ) {

}

/**
 * Moves the deltarobot to the starting position.
 **/
void DotMatrixPrinterNode::moveToStartPoint() {
	effectorLocation.x = 0;
	effectorLocation.y = 0;
	effectorLocation.z = -196.063;

	deltaRobotMoveToPoint(effectorLocation.x, effectorLocation.y, effectorLocation.z);
}

void DotMatrixPrinterNode::deltaRobotMoveToPoint(double x, double y, double z, double maxAcceleration){
	deltaRobotNode::MoveToPoint moveToPointService;
	moveToPointService.request.motion.x = x;
	moveToPointService.request.motion.y = y;
	moveToPointService.request.motion.z = z;
	moveToPointService.request.motion.maxAcceleration = maxAcceleration;

	deltaRobotClient.call(moveToPointService);
	if (!moveToPointService.response.succeeded) {
		std::cerr << "[ERROR] Moving to point (" << x << "," << y << "," << z << "," << maxAcceleration << ")" << moveToPointService.response.message << std::endl;
	}
}

/**
 * Draws a dot at coordinate x, y.
 * @param x X coordinate of the dotted pixel
 * @param y Y coordinate of the dotted pixel
 **/
void DotMatrixPrinterNode::drawDotToPath(double x, double y) {
	DataTypes::Point3D<double> oldLocation(effectorLocation);

	effectorLocation.x = x;
	effectorLocation.y = y;

	double elevation;
	if(oldLocation.distance(effectorLocation) > DotMatrixPrinterNodeSettings::MOVEMENT_THRESHOLD){
		elevation = DotMatrixPrinterNodeSettings::ELEVATION_BIG;
	} else {
		elevation = DotMatrixPrinterNodeSettings::ELEVATION_SMALL;
	}

	deltaRobotNode::Motion motion;
	motion.maxAcceleration = DotMatrixPrinterNodeSettings::ACCELERATION;

	motion.x = x;
	motion.y = y;
	
	motion.z = zDrawingSurface + elevation;
	movePathService.request.motion.push_back(motion);
	
	motion.z = zDrawingSurface;
	movePathService.request.motion.push_back(motion);
	
	motion.z = zDrawingSurface + elevation;
	movePathService.request.motion.push_back(motion);
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

	if(cols > DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH * DotMatrixPrinterNodeSettings::DRAW_FIELD_DOTS_PER_MM 
		|| rows > DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT * DotMatrixPrinterNodeSettings::DRAW_FIELD_DOTS_PER_MM){
		throw std::out_of_range("Invalid image size");
	}

	// Calculate offsets
	double offsetX = -(cols * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT / 2);
	double offsetY = rows  * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT / 2;

	moveToStartPoint();

	// Clear the old path.
	movePathService.request.motion.clear();

	double dotX, dotY;

	// moves left to right, right to left
	for(int row = 0; row < rows; row++){
		dotY = (-row + offsetY) * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT;
		if(row % 2 == 0){
			for(int col = 0; col < cols; col++){
				if (cv_ptr->image.data[row * cols + col] == 0) {
					dotX = (col + offsetX) * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT;
					drawDotToPath(dotX, dotY);
				}
			}
		} else {
			for(int col = cols - 1; col >= 0; col--){
				if (cv_ptr->image.data[row * cols + col] == 0) {
					dotX = (col + offsetX) * DotMatrixPrinterNodeSettings::DRAW_FIELD_MM_PER_DOT;
					drawDotToPath(dotX, dotY);
				}
			}
		}
	}

	Utilities::StopWatch stopwatch("PathTimer", true);
	movePathService.response.message = "";
	deltaRobotPathClient.call(movePathService);
	if (movePathService.response.message.compare("") != 0) {
		std::cout << "[ERROR]" << movePathService.response.message << std::endl;
	}
	stopwatch.stopAndPrint(stdout);

	moveToStartPoint();
}

void DotMatrixPrinterNode::calibrateDrawingHeight(){
	moveToStartPoint();
	
	enum CalibrationStep{
		PICK_COORDINATE = 0,
		TEST_CENTER = 1,
		TEST_CORNERS = 2
	};

	CalibrationStep step = PICK_COORDINATE;
	bool calibrated = false;

	while(ros::ok() && !calibrated){
		switch(step) {
			case PICK_COORDINATE: 
			{
				std::cout << "Calibrating pencil Z-axis. Enter desired coordinate:" << std::endl;
				std::string input;
				std::cin >> input;
				
				zDrawingSurface = Utilities::stringToDouble(input);

				step = TEST_CENTER;
				break;
			}
			case TEST_CENTER:
			{		
				std::cout << "Testing center: " << zDrawingSurface << std::endl;

				deltaRobotMoveToPoint(0, 0, zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint(0, 0, zDrawingSurface, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint(0, 0, zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);

				std::cout << "Enter R to repeat current, Y to accept, or anything else to try a new coordinate:" << std::endl;
				std::string input;
				std::cin >> input;

				if(input[0] == 'Y' || input[0] == 'y'){
					step = TEST_CORNERS;
				} else if(!(input[0] == 'R' || input[0] == 'r')){
					step = PICK_COORDINATE;
				}
				break;
			}
			case TEST_CORNERS:
			{
				std::cout << "Testing corners: " << zDrawingSurface << std::endl;
				
				deltaRobotMoveToPoint(-((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2), -((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2), zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint(-((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2), -((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2), zDrawingSurface, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint(-((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2), -((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2), zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);

				deltaRobotMoveToPoint((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, (double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2, zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, (double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2, zDrawingSurface, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, (double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2, zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);

				deltaRobotMoveToPoint((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, -((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2), zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, -((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2), zDrawingSurface, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2, -((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2), zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);

				deltaRobotMoveToPoint(-((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2), (double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2, zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint(-((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2), (double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2, zDrawingSurface, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);
				deltaRobotMoveToPoint(-((double)DotMatrixPrinterNodeSettings::DRAW_FIELD_WIDTH / 2), (double)DotMatrixPrinterNodeSettings::DRAW_FIELD_HEIGHT / 2, zDrawingSurface + DotMatrixPrinterNodeSettings::ELEVATION_BIG, DotMatrixPrinterNodeSettings::CALIBRATION_ACCELERATION);

				std::cout << "Enter R to repeat current, Y to accept, or anything else to try a new coordinate:" << std::endl;
				std::string input;
				std::cin >> input;

				if(input[0] == 'Y' || input[0] == 'y'){
					calibrated = true;
				} else if(!(input[0] == 'R' || input[0] == 'r')) {
					step = PICK_COORDINATE;
				} 
				break;
			}
		}
	}
}

/**
 * Blocking function that contains the main loop.
 * Spins in ROS to receive frames. These will execute the callbacks.
 * This function ends when ros receives a ^c
 **/
void DotMatrixPrinterNode::run( ) {
	calibrateDrawingHeight();

	moveToStartPoint();

	imageSubscriber = imageTransport.subscribe(ImageTransformationNodeTopics::TRANSFORMED_IMAGE, 1, &DotMatrixPrinterNode::imageCallback, this);
	std::cout << "DotMatrixPrinter started. Waiting for image on " << ImageTransformationNodeTopics::TRANSFORMED_IMAGE << std::endl;
	
	ros::Rate loopRate(1);

	while (ros::ok()) {
		ros::spinOnce();
		loopRate.sleep();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, NODE_NAME);

	DotMatrixPrinterNode dotMatrixPrinterNode;
	dotMatrixPrinterNode.run();

	return 0;
}

