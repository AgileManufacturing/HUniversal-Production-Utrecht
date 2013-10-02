/**
 * @file part_locator_node.cpp
 * @brief locates objects and rotates points.
 * @date Created: 2013-09-20
 *
 * @author Garik hakopian
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

#ifndef CAMERACALIBRATIONNODE_H_
#define CAMERACALIBRATIONNODE_H_

#include "ros/ros.h"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <qr_code_reader_node/Collection.h>

#include <iostream>

#include <Matrices/Matrices.h>
#include <Vectors/Vectors.h>


#include <rexos_blackboard_cpp_client/BlackboardCppClient.h>
#include <rexos_datatypes/InstructionData.h>


using namespace cv;

class PartLocatorNode {
	
public:
	PartLocatorNode(std::string blackboardIp);
	void run();
	void getRotationAngle();
private:
	Vector2 topLeftCoor;
	Vector2 topRightCoor;
	Vector2 bottomRightCoor;


		float currentXPos;
		float currentYPos;
		float currentZPos;
		std::string maxAcceleration;
		double STEP;

		Blackboard::BlackboardCppClient *equipletStepBlackboardClient;
		rexos_datatypes::InstructionData * instructionData;


	ros::NodeHandle nodeHandle;
	void qrCodeCallback(const qr_code_reader_node::Collection & message);

	void writeToBlackBoard(std::string x, std::string y, std::string z, std::string acceleration);
	
};

#endif /* CAMERACONTROLNODE_H_ */
