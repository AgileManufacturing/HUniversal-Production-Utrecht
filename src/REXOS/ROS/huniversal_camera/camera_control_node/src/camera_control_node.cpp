/**
 * @file CameraControlNode.cpp
 * @brief Remote interface to adjust the camera settings in runtime.
 * @date Created: 2012-10-18
 *
 * @author Tommas Bakker
 * @author Koen Braham
 * @author Daan Veltman
 *
 * @section LICENSE
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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

#include "camera_control_node/camera_control_node.h"

#include "vision_node/Services.h"
#include <vision_node/autoWhiteBalance.h>
#include <vision_node/enableComponent.h>
#include <vision_node/setCorrectionMatrices.h>
#include <vision_node/getCorrectionMatrices.h>
#include "camera_calibration_node/Services.h"
#include "camera_calibration_node/calibrateLens.h"
#include <rexos_knowledge_database/KnowledgeDatabaseException.h>
#include "rexos_utilities/Utilities.h"

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

#include <string>
#include <iostream>

CameraControlNode::CameraControlNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier) :
		rexos_module::Module::Module(equipletName, moduleIdentifier), 
		increaseExposureClient(			nodeHandle.serviceClient<std_srvs::Empty>(								vision_node_services::INCREASE_EXPOSURE)),
		decreaseExposureClient(			nodeHandle.serviceClient<std_srvs::Empty>(								vision_node_services::DECREASE_EXPOSURE)),
		autoWhiteBalanceClient(			nodeHandle.serviceClient<vision_node::autoWhiteBalance>(				vision_node_services::AUTO_WHITE_BALANCE)),
		fishEyeCorrectionClient(			nodeHandle.serviceClient<vision_node::enableComponent>(				vision_node_services::FISH_EYE_CORRECTION)),
		enableCameraClient(				nodeHandle.serviceClient<vision_node::enableComponent>(				vision_node_services::ENABLE_CAMERA)),
		getCorrectionMatricesClient(		nodeHandle.serviceClient<vision_node::getCorrectionMatrices>(		vision_node_services::GET_CORRECTION_MATRICES)),
		calibrateLensClient(				nodeHandle.serviceClient<camera_calibration_node::calibrateLens>(	camera_calibration_node_services::CALIBRATE_LENS))
{
}

void CameraControlNode::increaseExposureCall() {
	REXOS_INFO_STREAM("Calling service increaseExposureClient " << increaseExposureClient.call(emptyService));
}
void CameraControlNode::decreaseExposureCall() {
	REXOS_INFO_STREAM("Calling service decreaseExposureClient " << decreaseExposureClient.call(emptyService));
}
void CameraControlNode::autoWhiteBalanceCall(bool enabled) {
	vision_node::autoWhiteBalance autoWhiteBalance;
	autoWhiteBalance.request.enable = enabled;
	REXOS_INFO_STREAM("Calling service autoWhiteBalanceClient " << enabled << " " << autoWhiteBalanceClient.call(autoWhiteBalance));
}

void CameraControlNode::run() {
	ros::spin();
	char key;
	while(ros::ok() && !(key == 'q' || key == 'Q')) {
		std::cin >> key;

		if(key == 'a' || key == 'A') {
			autoWhiteBalanceCall(true);
		} else if(key == 'z' || key == 'Z') {
			autoWhiteBalanceCall(false);
		} else if(key == 's' || key == 'S') {
			increaseExposureCall();
		} else if(key == 'x' || key == 'X') {
			decreaseExposureCall();
		}
	}
}
bool CameraControlNode::mannuallyCalibrateLens(){
	// we have no calibration data for this combination, generate it
	// disable the fishEyeCorrection while doing the calibration
	vision_node::enableComponent fishEyeServiceCall;
	fishEyeServiceCall.request.enable = false;
	fishEyeCorrectionClient.call(fishEyeServiceCall);

	camera_calibration_node::calibrateLens calibrateLensServiceCall;
	calibrateLensServiceCall.request.frameCount = 20;
	calibrateLensServiceCall.request.boardWidth = 9;
	calibrateLensServiceCall.request.boardHeight = 6;
	calibrateLensClient.call(calibrateLensServiceCall);
	
	fishEyeServiceCall.request.enable = true;
	fishEyeCorrectionClient.call(fishEyeServiceCall);
	
	// did we see a chessboard?
	if(calibrateLensServiceCall.response.processedFrames == 0){
		REXOS_WARN_STREAM("calibrateLens failed, processedFrame = " << calibrateLensServiceCall.response.processedFrames);
		return false;
	}
	
	REXOS_INFO("Saving matrices to database");
	vision_node::getCorrectionMatrices getCalibrationMatricesServiceCall;
	getCorrectionMatricesClient.call(getCalibrationMatricesServiceCall);
	
	// copy all the distCoeffs from the service
	Json::Value distCoeffs;
	for(int i = 0; i < getCalibrationMatricesServiceCall.response.distCoeffs.size(); i++) {
		distCoeffs.append(getCalibrationMatricesServiceCall.response.distCoeffs.at(i));
	}
	// copy all the distCoeffs from the service
	Json::Value cameraMatrix;
	for(int i = 0; i < 9; i++) {
		cameraMatrix.append(getCalibrationMatricesServiceCall.response.cameraMatrix.values[i]);
	}
	
	// merge the distCoeffs and cameraMatrix in the already existing calibration data
	Json::Value jsonNode;
	try{
		std::string properties = this->getCalibrationDataForModuleAndChilds();
		Json::Reader reader;
		reader.parse(properties, jsonNode);
	} catch(rexos_knowledge_database::KnowledgeDatabaseException ex) {
		// there is no calibration data, so we start with a new node (which we created above)
	}
	jsonNode["distCoeffs"] = distCoeffs;
	jsonNode["cameraMatrix"] = cameraMatrix;
	
	Json::StyledWriter writer;
	REXOS_INFO_STREAM("JSON=" << writer.write(jsonNode));
	this->setCalibrationDataForModuleAndChilds(writer.write(jsonNode));
	return true;
}

bool CameraControlNode::transitionInitialize() {
	REXOS_INFO("Initialize transition called");
	return true;
}

bool CameraControlNode::transitionDeinitialize() {
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}

bool CameraControlNode::transitionSetup(){
	REXOS_INFO("Setup transition called");
	
	std::vector<rexos_datatypes::ModuleIdentifier> children = this->getChildModulesIdentifiers();
	if(children.size() != 1){
		REXOS_ERROR_STREAM("CameraControlNode::transitionSetup: Expected 1 child module (the lens), got " << children.size());
		return false;
	}
	
	try{
		std::string properties = this->getCalibrationDataForModuleAndChilds();
		REXOS_INFO(properties.c_str());
		Json::Value jsonNode;
		Json::Reader reader;
		reader.parse(properties, jsonNode);
		ROS_INFO("Trying transition setup a3");
		
		if(jsonNode.isMember("distCoeffs") == false || jsonNode.isMember("cameraMatrix") == false) {
			if(mannuallyCalibrateLens() == false){
				return false;
			}
		}
		
		Json::Value cameraMatrix = jsonNode["cameraMatrix"];
		Json::Value distCoeffs = jsonNode["distCoeffs"];
		if(cameraMatrix.size() != 9) {
			throw std::runtime_error("The camera matrix does not contain 9 entries" + boost::to_string(cameraMatrix.size()));
		}
		
		REXOS_INFO("calibrateLens correction matrix found");
		ros::ServiceClient client = nodeHandle.serviceClient<vision_node::setCorrectionMatrices>(vision_node_services::SET_CORRECTION_MATRICES);
		vision_node::setCorrectionMatrices serviceCall;
		
		// copy all the distCoeffs to the service
		for(int i = 0; i < distCoeffs.size(); i++) {
			serviceCall.request.distCoeffs.push_back(distCoeffs[i].asDouble());
		}
		// copy all the camera matrix to the service
		for(int i = 0; i < 9; i++) {
			serviceCall.request.cameraMatrix.values[i] = cameraMatrix[i].asDouble();
		}
		client.call(serviceCall);

		vision_node::enableComponent fishEyeServiceCall;
		fishEyeServiceCall.request.enable = true;
		fishEyeCorrectionClient.call(fishEyeServiceCall);

		return true;
	} catch(rexos_knowledge_database::KnowledgeDatabaseException ex) {
		if(mannuallyCalibrateLens() == false) return false;
		else return true;
	}
	
}
bool CameraControlNode::transitionShutdown(){
	REXOS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	return true;
}
bool CameraControlNode::transitionStart(){
	REXOS_INFO("Start transition called");
	vision_node::enableComponent serviceCall;
	serviceCall.request.enable = true;
	enableCameraClient.call(serviceCall);
	
	return true;
}
bool CameraControlNode::transitionStop(){
	REXOS_INFO("Stop transition called");
	vision_node::enableComponent serviceCall;
	serviceCall.request.enable = false;
	enableCameraClient.call(serviceCall);
	
	return true;
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "camera_control_node");
	
	if(argc < 5){
		REXOS_ERROR("Usage: gripper_node equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	std::string equipletName = argv[1];
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[2], argv[3], argv[4]);
	
	CameraControlNode node(equipletName, moduleIdentifier);
	
	REXOS_INFO_STREAM("Welcome to the camera node controller. Using this tool you can adjust camera settings on the fly :)."
	        << std::endl << "A\tEnable auto white balance" << std::endl << "Z\tDisable auto white balance" << std::endl
	        << "S\tIncrease exposure" << std::endl << "X\tDecrease exposure" << std::endl << "Q\tQuit program"
	        << std::endl << "Enter a key and press the \"Enter\" button" << std::endl);

	node.run();
	return 0;
}

