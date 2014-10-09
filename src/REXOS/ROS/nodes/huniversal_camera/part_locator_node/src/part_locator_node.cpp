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

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

#include <algorithm>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <rexos_statemachine/SetInstructionAction.h>

using namespace std;

const Vector2 PartLocatorNode::EXPECTED_DIRECTION = Vector2(-1, 0);
const Vector2 PartLocatorNode::EXPECTED_ITEM_DIRECTION = Vector2(-1, 0);
const int PartLocatorNode::minCornerSamples = 11;
const int PartLocatorNode::minItemSamples = 11;
/*const string PartLocatorNode::TOP_LEFT_VALUE = "WP_800_400_TL";
const string PartLocatorNode::TOP_RIGHT_VALUE = "WP_800_400_TR";
const string PartLocatorNode::BOTTOM_RIGHT_VALUE = "WP_800_400_BR";*/

PartLocatorNode::PartLocatorNode(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier):
		equipletName(equipletName),
		rexos_knowledge_database::Module(moduleIdentifier),
		rexos_coordinates::Module(this),
		rexos_statemachine::ModuleStateMachine(equipletName, moduleIdentifier, false),
		environmentCacheClient(nodeHandle.serviceClient<environment_cache::setData>("setData")),
		samplesTopLeft(minCornerSamples),
		samplesTopRight(minCornerSamples),
		samplesBottomRight(minCornerSamples)
{
	REXOS_INFO("Constructing");
	
	std::string properties = this->getModuleTypeProperties();
	
	Json::Reader reader;
	Json::Value jsonNode;
	reader.parse(properties, jsonNode);
	
	topLeftValue = jsonNode["topLeftValue"].asString();
	REXOS_INFO_STREAM("found topLeftValue " << topLeftValue);
	topRightValue = jsonNode["topRightValue"].asString();
	REXOS_INFO_STREAM("found topRightValue " << topRightValue);
	bottomRightValue = jsonNode["bottomRightValue"].asString();
	REXOS_INFO_STREAM("found bottomRightValue " << bottomRightValue);
	workPlaneWidth = jsonNode["workPlaneWidth"].asDouble();
	REXOS_INFO_STREAM("found workPlaneWidth " << workPlaneWidth);
	workSpaceHeight = jsonNode["workPlaneHeight"].asDouble();
	REXOS_INFO_STREAM("found workPlaneHeight " << workPlaneHeight);
	
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
		
			//REXOS_DEBUG_STREAM("QrCode \t" << message.qrCodes[i].value << " corner \t" << j);
			//if(j == 1) REXOS_INFO_STREAM("value: " << message.qrCodes[i].value);
			//if(j == 1) REXOS_INFO_STREAM("oldCoor: " << oldCoor);
			Vector3 newCoor = totalMatrix * oldCoor;
			//if(j == 1) REXOS_INFO_STREAM("newCoor: " << newCoor);
			newCoor = postCorrectionTotalMatrix * newCoor;
			//if(j == 1) REXOS_INFO_STREAM("newCoor: " << newCoor);
			//if(j == 1) REXOS_INFO_STREAM("eqlCoor: " << convertToEquipletCoordinate(newCoor));
			/*if(message.qrCodes[i].value == "GC4x4MB_1") {
				REXOS_DEBUG_STREAM("QrCode " << message.qrCodes[i].value << "\toldCoor \t" << oldCoor << "newCoor \t" << newCoor);
			}*/

			points[j] = Vector2(newCoor.x, newCoor.y);
		}
		Vector2 lineA2B = points[1] - points[0];
		Vector2 lineB2C = points[2] - points[1];
		
		//REXOS_DEBUG_STREAM("lineA2B \t" << lineA2B << " length " << lineA2B.length());
		//REXOS_DEBUG_STREAM("lineB2C \t" << lineB2C << " length " << lineB2C.length());
		
		QrCode qrCode;
		qrCode.location = Vector2(
			points[0].x + (lineA2B / 2).x + (lineB2C / 2).x,
			points[0].y + (lineA2B / 2).y + (lineB2C / 2).y
		);
		qrCode.angle = getItemRotationAngle(lineA2B);
		
		QrCode smoothCenterCoor = calculateSmoothPos(message.qrCodes[i].value, qrCode);
		
		Vector3 equipletCoor = convertToEquipletCoordinate(Vector3(smoothCenterCoor.location.x, smoothCenterCoor.location.y, 0));
		/*if(message.qrCodes[i].value == "GC4x4MB_1") {
			REXOS_DEBUG_STREAM("-equipletCoor \t" << equipletCoor);
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
	REXOS_DEBUG_STREAM("currentTopLeftCoor " << currentTopLeftCoor.location);
	REXOS_DEBUG_STREAM("currentTopRightCoor " << currentTopRightCoor.location);
	REXOS_DEBUG_STREAM("currentBottomRightCoor " << currentBottomRightCoor.location);
	
	bool updateMatrices = false;
	for(int i = 0; i < message.qrCodes.size(); i++) {
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
	
	//REXOS_DEBUG_STREAM("-expectedItemAngle \t" << expectedItemAngle);
	//REXOS_DEBUG_STREAM("-actualItemAngle \t" << actualItemAngle);
	/*if(message.qrCodes[i].value == "GC4x4MB_1") {
		REXOS_DEBUG_STREAM("QrCode " << message.qrCodes[i].value << "\t-angle \t" << angle);
	}*/
	return angle;
}
void PartLocatorNode::storeInEnviromentCache(std::string value, Vector3 location, double angle) {
		environment_cache::setData serviceCall;
		serviceCall.request.identifier = value;
		
		Json::Value data;
		Json::Value locationJson;
		locationJson["x"] = location.x;
		locationJson["y"] = location.y;
		locationJson["z"] = location.z;
		Json::Value rotationJson;
		rotationJson["z"] = angle;
		
		data["location"] = locationJson;
		data["rotation"] = rotationJson;
		
		Json::StyledWriter writer;
		serviceCall.request.json = writer.write(data);
		environmentCacheClient.call(serviceCall);
}

void PartLocatorNode::updateMatrices() {
	totalMatrix = calculateScaleMatrix() * calculateRotationMatrix() * calculateOffsetMatrix();
	//REXOS_INFO_STREAM("totalMatrix " << totalMatrix);
	
}
Matrix3 PartLocatorNode::calculateOffsetMatrix() {
	////////////
	// calulate midpoint
	////////////
	// line between topLeft and topRight
	Vector2 lineTl2Tr;
	lineTl2Tr.x = currentTopRightCoor.location.x - currentTopLeftCoor.location.x;
	lineTl2Tr.y = currentTopRightCoor.location.y - currentTopLeftCoor.location.y;
	REXOS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
	
	// line between topRight and bottomRight
	Vector2 lineTr2Br;
	lineTr2Br.x = currentBottomRightCoor.location.x - currentTopRightCoor.location.x;
	lineTr2Br.y = currentBottomRightCoor.location.y - currentTopRightCoor.location.y;
	REXOS_DEBUG_STREAM("lineTr2Br " << lineTr2Br);
	
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
	REXOS_DEBUG_STREAM("midpoint " << midPoint);
	
	Matrix3 translationMatrix;
	translationMatrix[2] = -midPoint.x;
	translationMatrix[5] = -midPoint.y;
	REXOS_DEBUG_STREAM("translationMatrix " << translationMatrix);
	
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
	REXOS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
	
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
	REXOS_DEBUG_STREAM("rotationMatrix " << rotationMatrix);

	REXOS_DEBUG_STREAM("actualDirection " << actualDirection);
	REXOS_DEBUG_STREAM("correctionAngle " << correctionAngle);
	
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
	REXOS_DEBUG_STREAM("lineTl2Tr " << lineTl2Tr);
	
	// line between topRight and bottomRight
	Vector2 lineTr2Br;
	lineTr2Br.x = currentBottomRightCoor.location.x - currentTopRightCoor.location.x;
	lineTr2Br.y = currentBottomRightCoor.location.y - currentTopRightCoor.location.y;
	REXOS_DEBUG_STREAM("lineTr2Br " << lineTr2Br);
	

	double workplateWidth = 80;
	double workplateHeight = 80;
	Matrix3 scaleMatrix;
	scaleMatrix[0] = -(	(workplateWidth / 1) / (lineTl2Tr.length()));
	scaleMatrix[4] = -(	(workplateHeight / 1) / (lineTr2Br.length()));
	REXOS_DEBUG_STREAM("lineTl2Tr.length() " << lineTl2Tr.length());
	REXOS_DEBUG_STREAM("lineTr2Br.length() " << lineTr2Br.length());
	REXOS_DEBUG_STREAM("scaleMatrix " << scaleMatrix);
	
	return scaleMatrix;
}

void PartLocatorNode::run() {
	REXOS_INFO("waiting for camera/qr_codes");
	ros::Subscriber sub = nodeHandle.subscribe("camera/qr_codes", 10, &PartLocatorNode::qrCodeCallback, this);
	ros::spin();
}
bool PartLocatorNode::transitionInitialize() {
	REXOS_INFO("Initialize transition called");
	return true;
}

bool PartLocatorNode::transitionDeinitialize() {
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}


bool PartLocatorNode::transitionSetup(){
	REXOS_INFO("Setup transition called");
	
	// @TODO Select either deltarobot or six_axisrobot (defaulted to sixaxis in constructor)
	actionlib::SimpleActionClient<rexos_statemachine::SetInstructionAction> setInstructionActionClient(nodeHandle, equipletName + "/HU/delta_robot_type_B/1/set_instruction");
	//  actionlib::SimpleActionClient<rexos_statemachine::SetInstructionAction> setInstructionActionClient(nodeHandle, equipletName + "/HU/six_axis_type_A/1/set_instruction");
	//actionlib::SimpleActionClient<rexos_statemachine::SetInstructionAction> setInstructionActionClient(nodeHandle, equipletName + "/HU/" +equipletIdentifier+"/1/set_instruction");
	std::string hardwareStep;
	rexos_statemachine::SetInstructionGoal* goal;
	
	
	workPlaneWidth = 80;
	workPlaneHeight = 80;
	workSpaceHeight = 50;
	
	/*rexos_statemachine::TransitionGoal goal2;
	std::vector<rexos_statemachine::RequiredMutation> requiredMutations;
	rexos_statemachine::RequiredMutation requiredMutation;
	requiredMutation.mutation = "move";
	requiredMutation.isOptional = false;
	requiredMutations.push_back(requiredMutation);
	goal2.requiredMutationsRequiredForNextPhase = requiredMutations;
	REXOS_INFO("SendGoal");
	transitionActionClient.sendGoal(goal2);
	transitionActionClient.waitForResult();
*/

	REXOS_INFO("Press enter when six-axis has finished calibration");
	cin.get();
	cin.ignore();


	REXOS_INFO("Continuing calibration");
	
	double x;
	double y;
	double z;
	int acceleration = 20;
	rexos_knowledge_database::Module drKnowMod = rexos_knowledge_database::Module(rexos_knowledge_database::ModuleIdentifier("HU", "delta_robot_type_B", "1"));
	//rexos_knowledge_database::Module drKnowMod = rexos_knowledge_database::Module(rexos_knowledge_database::ModuleIdentifier("HU", "six_axis_type_A", "1"));
	//rexos_knowledge_database::Module drKnowMod = rexos_knowledge_database::Module(rexos_knowledge_database::ModuleIdentifier("HU",equipletIdentifier, "1"));
	
	rexos_coordinates::Module drModule = rexos_coordinates::Module(&drKnowMod);
	Vector3 v;

	REXOS_INFO("Moving to top left corner");
	/*x = 0 - workPlaneWidth / 2;
	y = 0 + workPlaneHeight / 2;
	z = workSpaceHeight;
	v = Vector3(x, y, z);
	v = convertToEquipletCoordinate(v);
	v = drModule.convertToModuleCoordinate(v);
	rexos_datatypes::EquipletStep equipletStep;
	equipletStep.setModuleIdentifier(rexos_knowledge_database::ModuleIdentifier("HU", "delta_robot_type_B", "1"));
	
	rexos_datatypes::OriginPlacement originPlacement;
	originPlacement.setOriginPlacementType(rexos_datatypes::OriginPlacement::OriginPlacementType::RELATIVE_TO_MODULE_ORIGIN);
	equipletStep.setOriginPlacement(originPlacement);
	
	Json::Value moveCommand;
	moveCommand["x"] = v.x;
	moveCommand["x"] = v.y;
	moveCommand["x"] = v.z;
	Json::Value instructionData;
	instructionData["move"] = moveCommand;
	equipletStep.setInstructionData(instructionData);*/
	
	hardwareStep = "{\"command\":\"move\", \"look_up\":NULL, \"look_up_parameters\":NULL, \"payload\":{\"x\":" + boost::lexical_cast<std::string>(v.x) + ",\"y\":" + boost::lexical_cast<std::string>(v.y) + ",\"z\":" + boost::lexical_cast<std::string>(v.z) + ", \"maxAcceleration\": "+boost::lexical_cast<std::string>(acceleration) +"} }";
	REXOS_WARN_STREAM(hardwareStep);
	
	goal = new rexos_statemachine::SetInstructionGoal();
	goal->json = hardwareStep;
	goal->OID = 1;
	REXOS_WARN_STREAM(*goal);

	setInstructionActionClient.sendGoal(*goal);
	REXOS_INFO("enter diff to X");
	std::cin >> topLeftOffsetX;
	REXOS_INFO("enter diff to Y");
	std::cin >> topLeftOffsetY;
	
	REXOS_INFO("Moving to top right corner");
	x = 0 + workPlaneWidth / 2;
	y = 0 + workPlaneHeight / 2;
	z = workSpaceHeight;
	v = Vector3(x, y, z);
	v = convertToEquipletCoordinate(v);
	v = drModule.convertToModuleCoordinate(v);
	hardwareStep = "{\"command\":\"move\", \"look_up\":NULL, \"look_up_parameters\":NULL, \"payload\":{\"x\":" + boost::lexical_cast<std::string>(v.x) + ",\"y\":" + boost::lexical_cast<std::string>(v.y) + ",\"z\":" + boost::lexical_cast<std::string>(v.z) + ", \"maxAcceleration\":"+boost::lexical_cast<std::string>(acceleration) +" } }";
	REXOS_WARN_STREAM(hardwareStep);
	
	goal = new rexos_statemachine::SetInstructionGoal();
	goal->json = hardwareStep;
	goal->OID = 2;
	REXOS_WARN_STREAM(*goal);
	
	setInstructionActionClient.sendGoal(*goal);
	REXOS_INFO("enter diff to X");
	std::cin >> topRightOffsetX;
	REXOS_INFO("enter diff to Y");
	std::cin >> topRightOffsetY;
	
	REXOS_INFO("Moving to bottom right corner");
	x = 0 + workPlaneWidth / 2;
	y = 0 - workPlaneHeight / 2;
	z = workSpaceHeight;
	v = Vector3(x, y, z);
	v = convertToEquipletCoordinate(v);
	v = drModule.convertToModuleCoordinate(v);
	hardwareStep = "{\"command\":\"move\", \"look_up\":NULL, \"look_up_parameters\":NULL, \"payload\":{\"x\":" + boost::lexical_cast<std::string>(v.x) + ",\"y\":" + boost::lexical_cast<std::string>(v.y) + ",\"z\":" + boost::lexical_cast<std::string>(v.z) + ", \"maxAcceleration\":"+boost::lexical_cast<std::string>(acceleration) +" } }";
	REXOS_WARN_STREAM(hardwareStep);
	REXOS_WARN_STREAM(*goal);
		
	goal = new rexos_statemachine::SetInstructionGoal();
	goal->json = hardwareStep;
	goal->OID = 3;
	
	setInstructionActionClient.sendGoal(*goal);
	REXOS_INFO("enter diff to X");
	std::cin >> bottomRightOffsetX;
	REXOS_INFO("enter diff to Y");
	std::cin >> bottomRightOffsetY;
	
	Vector3 A = Vector3(0						, 0						, 1);
	Vector3 B = Vector3(0 + workPlaneWidth	, 0						, 1);
	Vector3 C = Vector3(0 + workPlaneWidth	, 0 - workPlaneHeight	, 1);
	
	Vector3 Aproj = Vector3(A.x + topLeftOffsetX, A.y + topLeftOffsetY, 1);
	Vector3 Bproj = Vector3(B.x + topRightOffsetX, B.y + topRightOffsetY, 1);
	Vector3 Cproj = Vector3(C.x + bottomRightOffsetX, C.y + bottomRightOffsetY, 1);
	
	REXOS_INFO_STREAM("workPlaneWidth " << workPlaneWidth << " workPlaneHeight " << workPlaneHeight);
	REXOS_INFO_STREAM("A " << A << " B " << B << " C " << C);
	REXOS_INFO_STREAM("Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	
	// fancy calulation here
	// first translate so that A' matches A
	REXOS_INFO_STREAM("x " << -topLeftOffsetX << " y " << -topLeftOffsetY);
	
	Matrix3 postCorrectionTranslationMatrix = Matrix3();
	postCorrectionTranslationMatrix[2] = -topLeftOffsetX;
	postCorrectionTranslationMatrix[5] = -topLeftOffsetY;
	
	Aproj = postCorrectionTranslationMatrix * Aproj;
	Bproj = postCorrectionTranslationMatrix * Bproj;
	Cproj = postCorrectionTranslationMatrix * Cproj;
	
	REXOS_INFO_STREAM(postCorrectionTranslationMatrix);
	REXOS_INFO_STREAM("translated: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	
	// second rotate so that normalize(A'B') matches normalize(AB)
	// calulate the angle between A' and B'
	double angleAprojBproj = std::atan2(Bproj.y - Aproj.y, Bproj.x - Aproj.x);
	// calulate the angle between A and B
	double angleAB = std::atan2(B.y - A.y, B.x - A.x); // should be 0
	double rotationAngle = angleAB - angleAprojBproj;
	REXOS_INFO_STREAM("angleAprojBproj " << angleAprojBproj << " angleAB " << angleAB << " rotationAngle " << rotationAngle);
	
	Matrix3 postCorrectionRotationMatrix = Matrix3();
	postCorrectionRotationMatrix[0] = cos(rotationAngle);
	postCorrectionRotationMatrix[1] = -sin(rotationAngle);
	postCorrectionRotationMatrix[3] = sin(rotationAngle);
	postCorrectionRotationMatrix[4] = cos(rotationAngle);
	
	Aproj = postCorrectionRotationMatrix * Aproj;
	Bproj = postCorrectionRotationMatrix * Bproj;
	Cproj = postCorrectionRotationMatrix * Cproj;
	
	REXOS_INFO_STREAM(postCorrectionRotationMatrix);
	REXOS_INFO_STREAM("rotated: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	
	// third shear so that normalize(B'C') mathces normalize(BC)
	// calulate the angle between A' and B'
	double angleBprojCproj = std::atan2(Cproj.y - Bproj.y, Cproj.x - Bproj.x);
	// calulate the angle between A and B
	double angleBC = std::atan2(C.y - B.y, C.x - B.x); // should be -1/2*Pi
	double shearAngle = angleBprojCproj - angleBC;
	double shearFactor = tan(shearAngle);
	ROS_INFO_STREAM("angleBprojCproj " << angleBprojCproj << " angleBC " << angleBC << 
			" shearAngle " << shearAngle << " shearFactor " << shearFactor);
	
	Matrix3 postCorrectionShearMatrix = Matrix3(
			1, shearFactor, 0,
			0, 1, 0,
			0, 0, 1);
	
	
	Aproj = postCorrectionShearMatrix * Aproj;
	Bproj = postCorrectionShearMatrix * Bproj;
	Cproj = postCorrectionShearMatrix * Cproj;
	
	REXOS_INFO_STREAM(postCorrectionShearMatrix);
	REXOS_INFO_STREAM("sheared: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	
	// fourth scale so that A'B' matches AB and B'C' matches BC
	Matrix3 postCorrectionScaleMatrix = Matrix3();
	double scaleX = (B - A).length() / (Bproj - Aproj).length();
	double scaleY = (C - B).length() / (Cproj - Bproj).length();
	postCorrectionScaleMatrix[0] = scaleX;
	postCorrectionScaleMatrix[4] = scaleY;
	REXOS_INFO_STREAM("scaleX: " << scaleX << " scaleY " << scaleY);
	
	Aproj = postCorrectionScaleMatrix * Aproj;
	Bproj = postCorrectionScaleMatrix * Bproj;
	Cproj = postCorrectionScaleMatrix * Cproj;
	
	REXOS_INFO_STREAM(postCorrectionScaleMatrix);
	REXOS_INFO_STREAM("scale: Aproj " << Aproj << " Bproj " << Bproj << " Cproj " << Cproj);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	
	Matrix3 translateToA = Matrix3(1, 0, workPlaneWidth / 2, 0, 1, -workPlaneHeight / 2, 0, 0, 1);
	Matrix3 translateFromA = Matrix3(1, 0, -workPlaneWidth / 2, 0, 1, workPlaneHeight / 2, 0, 0, 1);
	
	REXOS_INFO_STREAM(translateToA);
	REXOS_INFO_STREAM(translateFromA);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	
	postCorrectionTotalMatrix = translateFromA * postCorrectionScaleMatrix * 
			postCorrectionShearMatrix * postCorrectionRotationMatrix * 
			postCorrectionTranslationMatrix * translateToA;
	
	REXOS_INFO_STREAM(postCorrectionTotalMatrix);

	REXOS_INFO_STREAM("result: A " << A << "	->	 " << postCorrectionTotalMatrix * A);
	REXOS_INFO_STREAM("A " << A);
	REXOS_INFO_STREAM("A " << translateToA * A);
	REXOS_INFO_STREAM("A " << postCorrectionTranslationMatrix * translateToA * A);
	REXOS_INFO_STREAM("A " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
	REXOS_INFO_STREAM("A " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
	REXOS_INFO_STREAM("A " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
	REXOS_INFO_STREAM("A " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * A);
	REXOS_INFO_STREAM("result: B " << B << "	->	 " << postCorrectionTotalMatrix * B);
	REXOS_INFO_STREAM("B " << B);
	REXOS_INFO_STREAM("B " << translateToA * B);
	REXOS_INFO_STREAM("B " << postCorrectionTranslationMatrix * translateToA * B);
	REXOS_INFO_STREAM("B " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
	REXOS_INFO_STREAM("B " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
	REXOS_INFO_STREAM("B " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
	REXOS_INFO_STREAM("B " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * B);
	REXOS_INFO_STREAM("result: C " << C << "	->	 " << postCorrectionTotalMatrix * C);
	REXOS_INFO_STREAM("C " << C);
	REXOS_INFO_STREAM("C " << translateToA * C);
	REXOS_INFO_STREAM("C " << postCorrectionTranslationMatrix * translateToA * C);
	REXOS_INFO_STREAM("C " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
	REXOS_INFO_STREAM("C " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
	REXOS_INFO_STREAM("C " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
	REXOS_INFO_STREAM("C " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * C);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	
	
	x = 0 + workPlaneWidth / 2;
	y = 0 - workPlaneHeight / 2;
	v = Vector3(x, y, 1);
	REXOS_WARN_STREAM("--------------------------------------------------------------");
	REXOS_INFO_STREAM("v " << v);
	REXOS_INFO_STREAM("v " << translateToA * v);
	REXOS_INFO_STREAM("v " << postCorrectionTranslationMatrix * translateToA * v);
	REXOS_INFO_STREAM("v " << postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
	REXOS_INFO_STREAM("v " << postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
	REXOS_INFO_STREAM("v " << postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
	REXOS_INFO_STREAM("v " << translateFromA * postCorrectionScaleMatrix * postCorrectionShearMatrix * postCorrectionRotationMatrix * postCorrectionTranslationMatrix * translateToA * v);
	
	
	REXOS_INFO("Moving to top left corner");
	x = 0 - workPlaneWidth / 2;
	y = 0 + workPlaneHeight / 2;
	v = Vector3(x, y, 1);
	REXOS_INFO_STREAM(v);
	v = postCorrectionTotalMatrix * v;
	REXOS_INFO_STREAM(v);
	v.z = workSpaceHeight;
	v = convertToEquipletCoordinate(v);
	REXOS_INFO_STREAM(v);
	v = drModule.convertToModuleCoordinate(v);
	hardwareStep = "{\"command\":\"move\", \"look_up\":NULL, \"look_up_parameters\":NULL, \"payload\":{\"x\":" + boost::lexical_cast<std::string>(v.x) + ",\"y\":" + boost::lexical_cast<std::string>(v.y) + ",\"z\":" + boost::lexical_cast<std::string>(v.z) + ", \"maxAcceleration\":"+boost::lexical_cast<std::string>(acceleration) +" } }";
	
	goal = new rexos_statemachine::SetInstructionGoal();
	goal->json = hardwareStep;
	goal->OID = 1;
	setInstructionActionClient.sendGoal(*goal);
	// @TODO this might be removed?
	REXOS_INFO("Press enter when ready");
	cin.get();
	cin.ignore();
	
	REXOS_INFO("Moving to top right corner");
	x = 0 + workPlaneWidth / 2;
	y = 0 + workPlaneHeight / 2;
	v = Vector3(x, y, 1);
	REXOS_INFO_STREAM(v);
	v = postCorrectionTotalMatrix * v;
	REXOS_INFO_STREAM(v);
	v.z = workSpaceHeight;
	REXOS_INFO_STREAM(v);
	v = convertToEquipletCoordinate(v);
	REXOS_INFO_STREAM(v);
	v = drModule.convertToModuleCoordinate(v);
	hardwareStep = "{\"command\":\"move\", \"look_up\":NULL, \"look_up_parameters\":NULL, \"payload\":{\"x\":" + boost::lexical_cast<std::string>(v.x) + ",\"y\":" + boost::lexical_cast<std::string>(v.y) + ",\"z\":" + boost::lexical_cast<std::string>(v.z) + ", \"maxAcceleration\":"+boost::lexical_cast<std::string>(acceleration) +" } }";
	
	goal = new rexos_statemachine::SetInstructionGoal();
	goal->json = hardwareStep;
	goal->OID = 2;
	setInstructionActionClient.sendGoal(*goal);
	// @TODO this might be removed?
	REXOS_INFO("Press enter when ready");
	cin.get();
	cin.ignore();
	
	REXOS_INFO("Moving to bottom right corner");
	x = 0 + workPlaneWidth / 2;
	y = 0 - workPlaneHeight / 2;
	v = Vector3(x, y, 1);
	REXOS_INFO_STREAM(v);
	v = postCorrectionTotalMatrix * v;
	REXOS_INFO_STREAM(v);
	v.z = workSpaceHeight; //Changed from -15 to -10
	REXOS_INFO_STREAM(v);
	v = convertToEquipletCoordinate(v);
	REXOS_INFO_STREAM(v);
	v = drModule.convertToModuleCoordinate(v);
	hardwareStep = "{\"command\":\"move\", \"look_up\":NULL, \"look_up_parameters\":NULL, \"payload\":{\"x\":" + boost::lexical_cast<std::string>(v.x) + ",\"y\":" + boost::lexical_cast<std::string>(v.y) + ",\"z\":" + boost::lexical_cast<std::string>(v.z) + ", \"maxAcceleration\":"+boost::lexical_cast<std::string>(acceleration) +" } }";
		
	goal = new rexos_statemachine::SetInstructionGoal();
	goal->json = hardwareStep;
	goal->OID = 3;
	setInstructionActionClient.sendGoal(*goal);
	
	
	
	REXOS_INFO("Moving orign");
	x = 0;
	y = 0;
	v = Vector3(x, y, 1);
	REXOS_INFO_STREAM(v);
	v = postCorrectionTotalMatrix * v;
	REXOS_INFO_STREAM(v);
	v.z = workSpaceHeight;
	REXOS_INFO_STREAM(v);
	v = convertToEquipletCoordinate(v);
	REXOS_INFO_STREAM(v);
	v = drModule.convertToModuleCoordinate(v);
	
	hardwareStep = "{\"command\":\"move\", \"look_up\":NULL, \"look_up_parameters\":NULL, \"payload\":{\"x\":" + boost::lexical_cast<std::string>(v.x) + ",\"y\":" + boost::lexical_cast<std::string>(v.y) + ",\"z\":" + boost::lexical_cast<std::string>(v.z) + ", \"maxAcceleration\":"+boost::lexical_cast<std::string>(acceleration) +" } }";
	
	goal = new rexos_statemachine::SetInstructionGoal();
	goal->json = hardwareStep;
	goal->OID = 4;
	setInstructionActionClient.sendGoal(*goal);
	
	
	
	// @TODO this might be removed?
	REXOS_INFO("Press enter when ready");
	cin.get();
	cin.ignore();
	
	return true;
}
bool PartLocatorNode::transitionShutdown() {
	REXOS_INFO("Shutdown transition called");
	return true;
}
bool PartLocatorNode::transitionStart() {
	REXOS_INFO("Start transition called");
	return true;
}
bool PartLocatorNode::transitionStop() {
	REXOS_INFO("Stop transition called");
	return true;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "part_locator_node");
	
	std::string equipletName = argv[1];
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(argv[2], argv[3], argv[4]);
	
	REXOS_INFO("Creating PartLocatorNode");
	PartLocatorNode node(equipletName, moduleIdentifier);
	node.run();
	
	return 0;
}
