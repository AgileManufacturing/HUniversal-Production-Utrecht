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
#include "environment_cache/EnvironmentCache.h"

#include "rexos_utilities/Utilities.h"

#include <libjson/libjson.h>

#include <algorithm>
#include <vector>

using namespace std;

const Vector2 PartLocatorNode::EXPECTED_DIRECTION = Vector2(-1, 0);
const Vector2 PartLocatorNode::EXPECTED_ITEM_DIRECTION = Vector2(-1, 0);
const int PartLocatorNode::minCornerSamples = 101;
const int PartLocatorNode::minItemSamples = 11;
/*const string PartLocatorNode::TOP_LEFT_VALUE = "WP_800_400_TL";
const string PartLocatorNode::TOP_RIGHT_VALUE = "WP_800_400_TR";
const string PartLocatorNode::BOTTOM_RIGHT_VALUE = "WP_800_400_BR";*/


PartLocatorNode::PartLocatorNode(int equipletId, std::string cameraManufacturer, std::string cameraTypeNumber, std::string cameraSerialNumber):
		rexos_knowledge_database::Module(rexos_knowledge_database::ModuleIdentifier(cameraManufacturer, cameraTypeNumber, cameraSerialNumber)),
		rexos_coordinates::Module(this),
		environmentCacheClient(nodeHandle.serviceClient<environment_cache::UpdateEnvironmentCache>("updateEnvironmentCache")),
		samplesTopLeft(minCornerSamples),
		samplesTopRight(minCornerSamples),
		samplesBottomRight(minCornerSamples)
{
	ROS_INFO("Constructing");
	
	std::string properties = this->getModuleTypeProperties();
	JSONNode jsonNode = libjson::parse(properties);
	
	topLeftValue = std::string();
	topRightValue = std::string();
	bottomRightValue = std::string();
	double workPlaneWidth = std::numeric_limits<double>::quiet_NaN();
	double workPlaneHeight = std::numeric_limits<double>::quiet_NaN();
	
	for(JSONNode::const_iterator it = jsonNode.begin(); it != jsonNode.end(); it++) {
		if(it->name() == "topLeftValue"){
			topLeftValue = it->as_string();
			ROS_INFO_STREAM("found topLeftValue " << topLeftValue);
		} else if(it->name() == "topRightValue"){
			topRightValue = it->as_string();
			ROS_INFO_STREAM("found topRightValue " << topRightValue);
		} else if(it->name() == "bottomRightValue"){
			bottomRightValue = it->as_string();
			ROS_INFO_STREAM("found bottomRightValue " << bottomRightValue);
		} else if(it->name() == "workPlaneWidth"){
			workPlaneWidth = it->as_float();
			ROS_INFO_STREAM("found workPlaneWidth " << workPlaneWidth);
		} else if(it->name() == "workPlaneHeight"){
			workPlaneHeight = it->as_float();
			ROS_INFO_STREAM("found workPlaneHeight " << workPlaneHeight);
		} else {
			// some other property, ignore it
		}
	}
	if(std::isnan(workPlaneWidth) || std::isnan(workPlaneHeight) || 
			topLeftValue.length() == 0 || topRightValue.length() == 0 || bottomRightValue.length() == 0) {
		throw std::runtime_error("The properties do not contain the top/bottom left/right values or do not contain the workplane width/height");
	}
}

void PartLocatorNode::qrCodeCallback(const vision_node::QrCodes & message) {
	detectCorners(message);
	
	// can not do conversion without the corners
	if(
		samplesTopLeft.size() < minCornerSamples || 
		samplesTopRight.size() < minCornerSamples || 
		samplesBottomRight.size() < minCornerSamples) {
		return;
	}
	
	int collectionSize = message.qrCodes.size();
	for(int i = 0; i < collectionSize; i++){
		Vector2* points = new Vector2[3];
		for(int j = 0; j < 3; j++){
			Vector3 oldCoor;

			oldCoor.x = message.qrCodes[i].corners[j].x;
			oldCoor.y = message.qrCodes[i].corners[j].y;
			oldCoor.z = 1;
		
			//ROS_DEBUG_STREAM("QrCode \t" << message.qrCodes[i].value << " corner \t" << j);
			Vector3 newCoor = totalMatrix * oldCoor;
			/*if(message.qrCodes[i].value == "GC4x4MB_1") {
				ROS_DEBUG_STREAM("QrCode " << message.qrCodes[i].value << "\toldCoor \t" << oldCoor << "newCoor \t" << newCoor);
			}*/

			points[j] = Vector2(newCoor.x, newCoor.y);
		}
		Vector2 lineA2B = points[1] - points[0];
		Vector2 lineB2C = points[2] - points[1];
		
		//ROS_DEBUG_STREAM("lineA2B \t" << lineA2B << " length " << lineA2B.length());
		//ROS_DEBUG_STREAM("lineB2C \t" << lineB2C << " length " << lineB2C.length());
		
		QrCode qrCode;
		qrCode.location = Vector2(
			points[0].x + (lineA2B / 2).x + (lineB2C / 2).x,
			points[0].y + (lineA2B / 2).y + (lineB2C / 2).y
		);
		qrCode.angle = getItemRotationAngle(lineA2B);
		
		QrCode smoothCenterCoor = calculateSmoothPos(message.qrCodes[i].value, qrCode);
		
		Vector3 equipletCoor = convertToEquipletCoordinate(Vector3(smoothCenterCoor.location.x, smoothCenterCoor.location.y, 0));
		/*if(message.qrCodes[i].value == "GC4x4MB_1") {
			ROS_DEBUG_STREAM("-equipletCoor \t" << equipletCoor);
		}*/
		
		storeInEnviromentCache(message.qrCodes[i].value, equipletCoor, qrCode.angle);
		
		delete[] points;
	}
}
PartLocatorNode::QrCode PartLocatorNode::calculateSmoothPos(std::string name, PartLocatorNode::QrCode lastPosition) {
	boost::circular_buffer<QrCode> buffer;
	try{
		buffer = smoothBuffer.at(name);
	} catch(std::out_of_range ex) {
		buffer = boost::circular_buffer<QrCode>(minItemSamples);
	}
	buffer.push_back(lastPosition);
	
	return calculateSmoothPos(buffer);
}
PartLocatorNode::QrCode PartLocatorNode::calculateSmoothPos(boost::circular_buffer<PartLocatorNode::QrCode> buffer) {
	QrCode* points = buffer.linearize();
	std::vector<double> xArray;
	std::vector<double> yArray;
	std::vector<double> angleArray;
	for(int i = 0; i < buffer.size(); i++) {
		xArray.push_back(points[i].location.x);
		yArray.push_back(points[i].location.y);
		angleArray.push_back(points[i].angle);
	}
	
	double sumX = std::accumulate(xArray.begin(), xArray.end(), 0.0);
	double sumY = std::accumulate(yArray.begin(), yArray.end(), 0.0);
	double sumAngle = std::accumulate(angleArray.begin(), angleArray.end(), 0.0);
	
	QrCode output;
	output.location = Vector2(sumX / buffer.size(), sumY / buffer.size());
	output.angle = sumAngle / buffer.size();
	return output;
}

void PartLocatorNode::detectCorners(const vision_node::QrCodes & message) {
	ROS_DEBUG_STREAM("currentTopLeftCoor " << currentTopLeftCoor.location);
	ROS_DEBUG_STREAM("currentTopRightCoor " << currentTopRightCoor.location);
	ROS_DEBUG_STREAM("currentBottomRightCoor " << currentBottomRightCoor.location);
	
	bool updateMatrices = false;
	for(int i = 0; i < message.qrCodes.size(); i++){		
		QrCode qrCode;
		qrCode.location.x = message.qrCodes[i].corners[1].x;
		qrCode.location.y = message.qrCodes[i].corners[1].y;
		
		if(topLeftValue.compare(message.qrCodes[i].value) == 0){
			samplesTopLeft.push_back(qrCode);
			if(samplesTopLeft.size() == minCornerSamples - 1) {
				originalTopLeftCoor = calculateSmoothPos(samplesTopLeft);
			} else if(samplesTopLeft.size() == minCornerSamples) {
				currentTopLeftCoor = calculateSmoothPos(samplesTopLeft);
				updateMatrices = true;
			}
		} else if(topRightValue.compare(message.qrCodes[i].value) == 0){
			samplesTopRight.push_back(qrCode);
			if(samplesTopRight.size() == minCornerSamples - 1) {
				originalTopRightCoor = calculateSmoothPos(samplesTopRight);
			} else if(samplesTopRight.size() == minCornerSamples) {
				currentTopRightCoor = calculateSmoothPos(samplesTopRight);
				updateMatrices = true;
			}
		} else if(bottomRightValue.compare(message.qrCodes[i].value) == 0){
			samplesBottomRight.push_back(qrCode);
			if(samplesBottomRight.size() == minCornerSamples - 1) {
				originalBottomRightCoor = calculateSmoothPos(samplesBottomRight);
			} else if(samplesBottomRight.size() == minCornerSamples) {
				currentBottomRightCoor = calculateSmoothPos(samplesBottomRight);
				updateMatrices = true;
			}
		}
	}
	if(updateMatrices == true) this->updateMatrices();
}
double PartLocatorNode::getItemRotationAngle(Vector2 lineA2B) {
	Vector2 actualItemDirection(lineA2B);
	actualItemDirection.normalize();
	
	// calulate the expected angle (0)
	double expectedItemAngle = acos(EXPECTED_ITEM_DIRECTION.x);
	if(EXPECTED_ITEM_DIRECTION.y < 0) expectedItemAngle = 0 - expectedItemAngle;
	// calulate the actual angle (0)
	double actualItemAngle = acos(actualItemDirection.x);
	if(actualItemDirection.y < 0) actualItemAngle = 0 - actualItemAngle;
	
	double angle = actualItemAngle - expectedItemAngle;
	
	//ROS_DEBUG_STREAM("-expectedItemAngle \t" << expectedItemAngle);
	//ROS_DEBUG_STREAM("-actualItemAngle \t" << actualItemAngle);
	/*if(message.qrCodes[i].value == "GC4x4MB_1") {
		ROS_DEBUG_STREAM("QrCode " << message.qrCodes[i].value << "\t-angle \t" << angle);
	}*/
	return angle;
}
void PartLocatorNode::storeInEnviromentCache(std::string value, Vector3 location, double angle) {
		environment_cache::UpdateEnvironmentCache serviceCall;
		serviceCall.request.cacheUpdate.event = EnvironmentCache::ADD_OR_UPDATE;
		serviceCall.request.cacheUpdate.id = value;
		environment_communication_msgs::Map properties;
		environment_communication_msgs::KeyValuePair keyValuePair; 
		
		keyValuePair.key = "locationX";
		keyValuePair.value = boost::lexical_cast<string>(location.x);
		properties.map.push_back(environment_communication_msgs::KeyValuePair(keyValuePair));
		
		keyValuePair.key = "locationY";
		keyValuePair.value = boost::lexical_cast<string>(location.y);
		properties.map.push_back(environment_communication_msgs::KeyValuePair(keyValuePair));
		
		keyValuePair.key = "locationZ";
		keyValuePair.value = boost::lexical_cast<string>(location.z);
		properties.map.push_back(environment_communication_msgs::KeyValuePair(keyValuePair));
		
		keyValuePair.key = "angle";
		keyValuePair.value = boost::lexical_cast<string>(angle);
		properties.map.push_back(environment_communication_msgs::KeyValuePair(keyValuePair));
		
		serviceCall.request.cacheUpdate.properties = properties;
		environmentCacheClient.call(serviceCall);
}

void PartLocatorNode::updateMatrices() {
	totalMatrix = calculateScaleMatrix() * calculateRotationMatrix() * calculateOffsetMatrix();
	ROS_INFO_STREAM("totalMatrix " << totalMatrix);
	
}
Matrix3 PartLocatorNode::calculateOffsetMatrix() {
	////////////
	// calulate midpoint
	////////////
	// line between topLeft and topRight
	Vector2 lineTl2Tr;
	lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
	lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
	ROS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
	
	// line between topRight and bottomRight
	Vector2 lineTr2Br;
	lineTr2Br.x = currentBottomRightCoor.location.x - currentTopRightCoor.location.x;
	lineTr2Br.y = currentBottomRightCoor.location.y - currentTopRightCoor.location.y;
	ROS_DEBUG_STREAM("lineTr2Br " << lineTr2Br);
	
	// calulate new midpoint by deviding lineTl2Tr and bottomRight and then adding them
	Vector2 halfLineTl2Tr;
	halfLineTl2Tr.x = lineTl2Tr.x / 2;
	halfLineTl2Tr.y = lineTl2Tr.y / 2;
	Vector2 halfLineTr2Br;
	halfLineTr2Br.x = lineTr2Br.x / 2;
	halfLineTr2Br.y = lineTr2Br.y / 2;
	
	Vector2 midPoint;
	midPoint.x = currentTopLeftCoor.location.x + halfLineTl2Tr.x + halfLineTr2Br.x;
	midPoint.y = currentTopLeftCoor.location.y + halfLineTl2Tr.y + halfLineTr2Br.y;
	ROS_DEBUG_STREAM("midpoint " << midPoint);
	
	Matrix3 translationMatrix;
	translationMatrix[2] = -midPoint.x;
	translationMatrix[5] = -midPoint.y;
	ROS_DEBUG_STREAM("translationMatrix " << translationMatrix);
	
	return translationMatrix;
}
Matrix3 PartLocatorNode::calculateRotationMatrix() {
	////////////
	// calulate rotation angle
	////////////
	// line between topLeft and topRight
	Vector2 lineTl2Tr;
	lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
	lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
	ROS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
	
	// the expected vector is horizontal to the left because TR is at the right of TL (eg. walk in this direction to get to TR)
	Vector2 actualDirection;
	actualDirection.x = lineTl2Tr.x;
	actualDirection.y = lineTl2Tr.y;
	actualDirection.normalize();
	
	// we could calulate the angle by calulating the dot product and convert it to radians. But this is a more fancy approach (it actually aint)
	// calulate the expected angle (0)
	double expectedAngle = acos(EXPECTED_DIRECTION.x);
	if(EXPECTED_DIRECTION.y < 0) expectedAngle = 0 - expectedAngle;
	
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
	ROS_DEBUG_STREAM("rotationMatrix " << rotationMatrix);

	ROS_DEBUG_STREAM("actualDirection " << actualDirection);
	ROS_DEBUG_STREAM("correctionAngle " << correctionAngle);
	
	return rotationMatrix;
}
Matrix3 PartLocatorNode::calculateScaleMatrix() {
	////////////
	// scale to workplate coor system
	////////////
	// line between topLeft and topRight
	Vector2 lineTl2Tr;
	lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
	lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
	ROS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
	
	// line between topRight and bottomRight
	Vector2 lineTr2Br;
	lineTr2Br.x = currentBottomRightCoor.location.x - currentTopRightCoor.location.x;
	lineTr2Br.y = currentBottomRightCoor.location.y - currentTopRightCoor.location.y;
	ROS_DEBUG_STREAM("lineTr2Br " << lineTr2Br);
	

	double workplateWidth = 80;
	double workplateHeight = 80;
	Matrix3 scaleMatrix;
	scaleMatrix[0] = -(	(workplateWidth / 1) / (lineTl2Tr.length()));
	scaleMatrix[4] = -(	(workplateHeight / 1) / (lineTr2Br.length()));
	ROS_DEBUG_STREAM("lineTl2Tr.length() " << lineTl2Tr.length());
	ROS_DEBUG_STREAM("lineTr2Br.length() " << lineTr2Br.length());
	ROS_DEBUG_STREAM("scaleMatrix " << scaleMatrix);
	
	return scaleMatrix;
}

void PartLocatorNode::run() {
	ROS_INFO("waiting for camera/qr_codes");
	ros::Subscriber sub = nodeHandle.subscribe("camera/qr_codes", 10, &PartLocatorNode::qrCodeCallback, this);
	
	ros::spin();
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "part_locator_node");
	
	if(argc < 5){
		ROS_ERROR("Usage: camera_control_node equipletId, manufacturer, typeNumber, serialNumber");
		return -1;
	}
	
	int equipletId;
	try{
		equipletId = rexos_utilities::stringToInt(argv[1]);
	} catch(std::runtime_error ex) {
		ROS_ERROR("Cannot read equiplet id from commandline please use correct values.");
		return -2;
	}
	ROS_INFO("Constructing node");
	PartLocatorNode node(equipletId, argv[2], argv[3], argv[4]);
	
	node.run();
	return 0;
}
