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
#include "ros/ros.h"

#include "part_locator_node/part_locator_node.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <qr_code_reader_node/Collection.h>

#include <iostream>
#include <string>




#include "rexos_utilities/Utilities.h"



using namespace cv;
using namespace std;

struct qrCode {
	Point2f points[3];
} ;



PartLocatorNode::PartLocatorNode(std::string blackboardIp):
	currentXPos(0.0),
	currentYPos(0.0),
	currentZPos(0.0),
	maxAcceleration("50.0"),
	STEP(1.0) {

	ROS_INFO("Constructing");

	//DirectMoveStepsBlackBoard
	equipletStepBlackboardClient = new Blackboard::BlackboardCppClient(blackboardIp, "EQ1", "DirectMoveStepsBlackBoard");

	equipletStepBlackboardClient->removeDocuments("");
}

void PartLocatorNode::qrCodeCallback(const qr_code_reader_node::Collection & message){
	string topLeftValue = "WP_800_400_TL";
	string topRightValue = "WP_800_400_TR";
	string bottomRightValue = "WP_800_400_BR";
	
	int collectionSize = message.collection.size();
	int foundCorners = 0;
	
	for(int i = 0; i < collectionSize; i++){		
		if(topLeftValue.compare(message.collection[i].value) == 0){
			topLeftCoor.x = message.collection[i].corners[1].x;
			topLeftCoor.y = message.collection[i].corners[1].y;
foundCorners++;			
		}else if(topRightValue.compare(message.collection[i].value) == 0){
			topRightCoor.x = message.collection[i].corners[1].x;
			topRightCoor.y = message.collection[i].corners[1].y;			
foundCorners++;			
		}else if(bottomRightValue.compare(message.collection[i].value) == 0){
			bottomRightCoor.x = message.collection[i].corners[1].x;
			bottomRightCoor.y = message.collection[i].corners[1].y;			
foundCorners++;			
		}
	}






	ROS_INFO_STREAM("topLeftCoor " << topLeftCoor);
	ROS_INFO_STREAM("topRightCoor " << topRightCoor);
	ROS_INFO_STREAM("bottomRightCoor " << bottomRightCoor);
	
	////////////
	// calulate midpoint
	////////////
	// line between topLeft and topRight
	Vector2 lineTl2Tr;
	lineTl2Tr.x = topRightCoor.x - topLeftCoor.x;
	lineTl2Tr.y = topRightCoor.y - topLeftCoor.y;
	//ROS_INFO_STREAM("lineTl2Tr " << lineTl2Tr);
	
	// line between topRight and bottomRight
	Vector2 lineTr2Br;
	lineTr2Br.x = bottomRightCoor.x - topRightCoor.x;
	lineTr2Br.y = bottomRightCoor.y - topRightCoor.y;
	//ROS_INFO_STREAM("lineTr2Br " << lineTr2Br);
	
	// calulate new midpoint by deviding lineTl2Tr and bottomRight and then adding them
	Vector2 halfLineTl2Tr;
	halfLineTl2Tr.x = lineTl2Tr.x / 2;
	halfLineTl2Tr.y = lineTl2Tr.y / 2;
	Vector2 halfLineTr2Br;
	halfLineTr2Br.x = lineTr2Br.x / 2;
	halfLineTr2Br.y = lineTr2Br.y / 2;
	
	Vector2 midPoint;
	midPoint.x = topLeftCoor.x + halfLineTl2Tr.x + halfLineTr2Br.x;
	midPoint.y = topLeftCoor.y + halfLineTl2Tr.y + halfLineTr2Br.y;
	//ROS_INFO_STREAM("midpoint " << midPoint);
	
	Matrix3 translationMatrixA;
	translationMatrixA[2] = -midPoint.x;
	translationMatrixA[5] = -midPoint.y;
	Matrix3 translationMatrixB;
	translationMatrixB[2] = midPoint.x;
	translationMatrixB[5] = midPoint.y;
	//ROS_INFO_STREAM("translationMatrixA " << translationMatrixA);
	//ROS_INFO_STREAM("translationMatrixB " << translationMatrixB);
	
	
	////////////
	// calulate rotation angle
	////////////
	// the expected vector is horizontal to the left because TR is at the right of TL (eg. walk in this direction to get to TR)
	Vector2 expectedDirection(-1, 0); 
	Vector2 actualDirection;
	actualDirection.x = lineTl2Tr.x;
	actualDirection.y = lineTl2Tr.y;
	actualDirection.normalize();
	
	// we could calulate the angle by calulating the dot product and convert it to radians. But this is a more fancy approach (it actually aint)
	// calulate the expected angle (0)
	double expectedAngle = acos(expectedDirection.x);
	if(expectedDirection.y < 0) expectedAngle = 0 - expectedAngle;
	
	// calulate the actual angle
	double actualAngle = acos(actualDirection.x);
	if(actualDirection.y < 0) actualAngle = 0 - actualAngle;
	
	// substract the two angles to get the correction angle
	double correctionAngle = expectedAngle - actualAngle;
	
	Matrix3 rotationMatrix;
	rotationMatrix[0] = cos(correctionAngle);
	rotationMatrix[1] = -sin(correctionAngle);
	rotationMatrix[3] = sin(correctionAngle);
	rotationMatrix[4] = cos(correctionAngle);
/*	ROS_INFO_STREAM("rotationMatrix " << rotationMatrix);

	ROS_INFO_STREAM("expectedDirection " << expectedDirection);
	ROS_INFO_STREAM("actualDirection " << actualDirection);
	ROS_INFO_STREAM("correctionAngle " << correctionAngle);*/
	
	////////////
	// scale to workplate coor system
	////////////
	double workplateWidth = 80;
	double workplateHeight = 80;
	Matrix3 scaleMatrix;
	scaleMatrix[0] = -(	(workplateWidth / 1) / (lineTl2Tr.length()));
	scaleMatrix[4] = -(	(workplateHeight / 1) / (lineTr2Br.length()));
	/*ROS_INFO_STREAM("lineTl2Tr.length() " << lineTl2Tr.length());
	ROS_INFO_STREAM("lineTr2Br.length() " << lineTr2Br.length());
	ROS_INFO_STREAM("scaleMatrix " << scaleMatrix);
	
	Matrix3 totalMatrix = scaleMatrix * rotationMatrix * translationMatrixA;
	ROS_INFO_STREAM("totalMatrix " << totalMatrix);*/
	
	
	
	
	
	
	
	
	
	
	
	
/*	for(int i = 0; i < collectionSize; i++){
		if(message.collection[i].value == "GC4x4MB_1"){
			Vector2* points = new Vector2[3];
			for(int j = 0; j < 3; j++){
				Vector3 oldVector;

				oldVector.x = message.collection[i].corners[j].x;
				oldVector.y = message.collection[i].corners[j].y;
				oldVector.z = 1;
			
				ROS_INFO_STREAM("QrCode \t" << message.collection[i].value << " corner \t" << j);
				Vector3 newCoor = oldVector;
	//			ROS_INFO_STREAM("-old \t\t" << newCoor);
				newCoor = translationMatrixA * newCoor;
	//			ROS_INFO_STREAM("-transCoor \t" << newCoor);
				newCoor = rotationMatrix * newCoor;
	//			ROS_INFO_STREAM("rotCoor \t" << newCoor);
				newCoor = scaleMatrix * newCoor;
				ROS_INFO_STREAM("-scaleCoor \t" << newCoor);

				points[j] = Vector2(newCoor.x, newCoor.y);
			
			}
			
			Vector2 lineA2B = points[1] - points[0];
			Vector2 lineB2C = points[2] - points[1];
		
			ROS_INFO_STREAM("lineA2B \t" << lineA2B << " length " << lineA2B.length());
			ROS_INFO_STREAM("lineB2C \t" << lineB2C << " length " << lineB2C.length());

			Vector2 centerCoor = Vector2(
				points[0].x + (lineA2B / 2).x + (lineB2C / 2).x,
				points[0].y + (lineA2B / 2).y + (lineB2C / 2).y
			);

			if(abs(centerCoor.x) < 40 && abs(centerCoor.y) < 40 && abs(centerCoor.x) > 0 && abs(centerCoor.y) > 0){
				currentXPos = centerCoor.x;
				currentYPos = centerCoor.y;

				currentZPos = 0;
				}
			else{
				ROS_INFO(" FU ");
				currentXPos = 0;
				currentYPos = 0;

				currentZPos = 70;
			}
			delete points;
		}
	}


	writeToBlackBoard(rexos_utilities::doubleToString(currentXPos), rexos_utilities::doubleToString(currentYPos), rexos_utilities::doubleToString(currentZPos), maxAcceleration);*/



}
	













void PartLocatorNode::writeToBlackBoard(std::string x, std::string y, std::string z, std::string acceleration){
	/*//Need to include InstructionData for the instructiondata
	//Dont forget to set in package & makelist!

	std::map<std::string, std::string> look_up_parameters;
	std::map<std::string, std::string> payload;

	look_up_parameters.insert(pair<string, string>("ID", "RELATIVE-TO-PLACEHOLDER"));

	if(!x.empty()) {
		payload.insert(pair<string, string>("x", x));
	}

	if(!y.empty()) {
		payload.insert(pair<string, string>("y", y));
	}

	if(!z.empty()) {
		payload.insert(pair<string, string>("z", z));
	}

	if(!acceleration.empty()) {
		payload.insert(pair<string, string>("maxAcceleration", acceleration));
	}

	instructionData = new rexos_datatypes::InstructionData("move", "deltarobot", "FIND_ID", 
            look_up_parameters, payload);

	if(equipletStepBlackboardClient->insertDocument(instructionData->toJSONString())) {
		std::cout << "printed: " << instructionData->toJSONString() << "to blackboard." << std::endl;
	}*/

}





void PartLocatorNode::run() {
	ROS_INFO("waiting for camera/qr_codes");
	ros::Subscriber sub = nodeHandle.subscribe("camera/qr_codes", 10, &PartLocatorNode::qrCodeCallback,this);
	
	ros::spin();
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "part_locator_node");
	ROS_DEBUG("Constructing node");

	PartLocatorNode node("145.89.191.131");
	
	node.run();
	return 0;
}
