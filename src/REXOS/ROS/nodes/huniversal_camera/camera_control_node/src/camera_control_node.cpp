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

#include "camera_node/Services.h"
#include "camera_node/fishEyeCorrection.h"
#include "camera_node/enableCamera.h"
#include "camera_node/getCorrectionMatrices.h"
#include "camera_calibration_node/Services.h"
#include "camera_calibration_node/calibrateLens.h"
#include "rexos_utilities/Utilities.h"

#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <iostream>

CameraControlNode::CameraControlNode(int equipletId, int cameraModuleId, int lensModuleId) :
		increaseExposureClient(			nodeHandle.serviceClient<std_srvs::Empty>(							camera_node_services::INCREASE_EXPOSURE)),
		decreaseExposureClient(			nodeHandle.serviceClient<std_srvs::Empty>(							camera_node_services::DECREASE_EXPOSURE)),
		autoWhiteBalanceClient(			nodeHandle.serviceClient<camera_node::AutoWhiteBalance>(			camera_node_services::AUTO_WHITE_BALANCE)),
		fishEyeCorrectionClient(		nodeHandle.serviceClient<camera_node::fishEyeCorrection>(			camera_node_services::FISH_EYE_CORRECTION)),
		enableCameraClient(				nodeHandle.serviceClient<camera_node::enableCamera>(				camera_node_services::ENABLE_CAMERA)),
		getCorrectionMatricesClient(	nodeHandle.serviceClient<camera_node::getCorrectionMatrices>(		camera_node_services::GET_CORRECTION_MATRICES)),
		calibrateLensClient(			nodeHandle.serviceClient<camera_calibration_node::calibrateLens>(	camera_calibration_node_services::CALIBRATE_LENS)),
		rexos_statemachine::ModuleStateMachine("camera_control_node",equipletId, cameraModuleId, true),
		equipletId(equipletId),
		cameraModuleId(cameraModuleId),
		lensModuleId(lensModuleId)
{
}

void CameraControlNode::increaseExposureCall() {
	ROS_INFO_STREAM("Calling service increaseExposureClient " << increaseExposureClient.call(emptyService));
}

void CameraControlNode::decreaseExposureCall() {
	ROS_INFO_STREAM("Calling service decreaseExposureClient " << decreaseExposureClient.call(emptyService));
}

void CameraControlNode::autoWhiteBalanceCall(bool enabled) {
	camera_node::AutoWhiteBalance autoWhiteBalance;
	autoWhiteBalance.request.enable = enabled;
	ROS_INFO_STREAM("Calling service autoWhiteBalanceClient " << enabled << " " << autoWhiteBalanceClient.call(autoWhiteBalance));
}

void CameraControlNode::run() {
	ros::spin();
/*	char key;
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
	}*/
}
void CameraControlNode::transitionSetup(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Setup transition called");
	
	std::auto_ptr<sql::Connection> apConnection = rexos_knowledge_database::connect();
	
	// stmt = statement
	// pstmt = prepared statement
	sql::PreparedStatement* selectCalibrationMatricesPStmt = apConnection->prepareStatement("\
	SELECT calibrationmatrices.* FROM calibrationmatrices \
	JOIN modules AS lensModules ON lensModules.module_type = calibrationMatrices.lens \
	JOIN modules AS cameraModules ON cameraModules.module_type = calibrationMatrices.camera \
	WHERE lensModules.id = ? AND cameraModules.id = ?;");
	selectCalibrationMatricesPStmt->setInt(1, lensModuleId);
	selectCalibrationMatricesPStmt->setInt(2, cameraModuleId);
	
	sql::ResultSet* calibrationMatricesResult = selectCalibrationMatricesPStmt->executeQuery();
	
	if(calibrationMatricesResult->rowsCount() == 1){
		// we have found a calibrateLens correction matrix
		ROS_INFO("calibrateLens correction matrix found");
		
		ros::ServiceClient client = nodeHandle.serviceClient<camera_node::setCorrectionMatrices>(camera_node_services::SET_CORRECTION_MATRICES);
		camera_node::setCorrectionMatrices serviceCall;
		
		// set the cursor at the first result
		calibrationMatricesResult->next(); 
		
		// yeah, this is quite ugly
		serviceCall.request.distCoeffs.push_back(calibrationMatricesResult->getDouble("distCoef_0"));
		serviceCall.request.distCoeffs.push_back(calibrationMatricesResult->getDouble("distCoef_1"));
		serviceCall.request.distCoeffs.push_back(calibrationMatricesResult->getDouble("distCoef_2"));
		serviceCall.request.distCoeffs.push_back(calibrationMatricesResult->getDouble("distCoef_3"));
		serviceCall.request.distCoeffs.push_back(calibrationMatricesResult->getDouble("distCoef_4"));
	
		serviceCall.request.cameraMatrix.values[0] = calibrationMatricesResult->getDouble("cameraMatrix_0_0");
		serviceCall.request.cameraMatrix.values[1] = calibrationMatricesResult->getDouble("cameraMatrix_0_1");
		serviceCall.request.cameraMatrix.values[2] = calibrationMatricesResult->getDouble("cameraMatrix_0_2");
		serviceCall.request.cameraMatrix.values[3] = calibrationMatricesResult->getDouble("cameraMatrix_1_0");
		serviceCall.request.cameraMatrix.values[4] = calibrationMatricesResult->getDouble("cameraMatrix_1_1");
		serviceCall.request.cameraMatrix.values[5] = calibrationMatricesResult->getDouble("cameraMatrix_1_2");
		serviceCall.request.cameraMatrix.values[6] = calibrationMatricesResult->getDouble("cameraMatrix_2_0");
		serviceCall.request.cameraMatrix.values[7] = calibrationMatricesResult->getDouble("cameraMatrix_2_1");
		serviceCall.request.cameraMatrix.values[8] = calibrationMatricesResult->getDouble("cameraMatrix_2_2");
		
		client.call(serviceCall);
		
		as->setSucceeded();
	}
	else{
		// disable the fishEyeCorrection while doing the calibration
		camera_node::fishEyeCorrection fishEyeServiceCall;
		fishEyeServiceCall.request.enable = false;
		fishEyeCorrectionClient.call(fishEyeServiceCall);

		camera_calibration_node::calibrateLens calibrateLensServiceCall;
		calibrateLensServiceCall.request.frameCount = 20;
		calibrateLensServiceCall.request.boardWidth = 9;
		calibrateLensServiceCall.request.boardHeight = 6;
		calibrateLensClient.call(calibrateLensServiceCall);
		
		fishEyeServiceCall.request.enable = true;
		fishEyeCorrectionClient.call(fishEyeServiceCall);
		
		// dit we see a chessboard?
		if(calibrateLensServiceCall.response.processedFrames == 0){
			ROS_WARN_STREAM("calibrateLens failed, processedFrame = " << calibrateLensServiceCall.response.processedFrames);
			as->setAborted();
		}
		else{
			ROS_INFO_STREAM("Saving matrices to database for cameraId=" << cameraModuleId << " and lensModuleId=" << lensModuleId);
			camera_node::getCorrectionMatrices getCalibrationMatricesServiceCall;
			getCorrectionMatricesClient.call(getCalibrationMatricesServiceCall);
			
			sql::PreparedStatement* insertCalibrationMatricesPStmt = apConnection->prepareStatement(
			"INSERT INTO calibrationmatrices ( \
				camera, \
				lens, \
				distCoef_0,  distCoef_1,  distCoef_2, distCoef_3,  distCoef_4, \
				cameraMatrix_0_0, cameraMatrix_0_1, cameraMatrix_0_2, \
				cameraMatrix_1_0, cameraMatrix_1_1, cameraMatrix_1_2, \
				cameraMatrix_2_0, cameraMatrix_2_1, cameraMatrix_2_2 \
			) VALUES ( \
				(SELECT module_type FROM modules WHERE id = ?), (SELECT module_type FROM modules WHERE id = ?), \
				(SELECT module_type FROM modules WHERE id = ?), (SELECT module_type FROM modules WHERE id = ?), \
				?, ?, ?, ?, ?, \
				?, ?, ?, \
				?, ?, ?, \
				?, ?, ? \
			);");
			insertCalibrationMatricesPStmt->setInt(1, cameraModuleId);
			insertCalibrationMatricesPStmt->setInt(2, lensModuleId);
			
			int collumIndex = 3;
			// push distortion coeffs
			for(int i = 0; i < getCalibrationMatricesServiceCall.response.distCoeffs.size(); i++) {
				insertCalibrationMatricesPStmt->setDouble(collumIndex, getCalibrationMatricesServiceCall.response.distCoeffs[i]);
				collumIndex++;
			}
			// push camera matrix
			for(int i = 0; i < 9; i++) {
				insertCalibrationMatricesPStmt->setDouble(collumIndex, getCalibrationMatricesServiceCall.response.cameraMatrix.values[i]);
				collumIndex++;
			}
			insertCalibrationMatricesPStmt->executeQuery();
			
			as->setSucceeded();
		}
	}

}
void CameraControlNode::transitionShutdown(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	as->setSucceeded();
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
void CameraControlNode::transitionStart(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Start transition called");
	camera_node::enableCamera serviceCall;
	serviceCall.request.enable = true;
	enableCameraClient.call(serviceCall);
	
	as->setSucceeded();
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
void CameraControlNode::transitionStop(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Stop transition called");
	camera_node::enableCamera serviceCall;
	serviceCall.request.enable = false;
	enableCameraClient.call(serviceCall);
	
	as->setSucceeded();
}


int main(int argc, char* argv[]) {
	ros::init(argc, argv, "cameraControlNode");
	
	if(argc < 3){
		ROS_ERROR("Arguments are not valid");
		return -1;
	}
	
	int equipletId, cameraModuleId, lensModuleId;
	try{
		equipletId = rexos_utilities::stringToInt(argv[1]);
		cameraModuleId = rexos_utilities::stringToInt(argv[2]);
		lensModuleId = rexos_utilities::stringToInt(argv[3]);
	} catch(std::runtime_error ex) {
		ROS_ERROR("Cannot read equiplet id, camera id, and/or lens id from commandline please use correct values.");
		return -2;
	}

	CameraControlNode node(equipletId, cameraModuleId, lensModuleId);

	std::cout << "Welcome to the camera node controller. Using this tool you can adjust camera settings on the fly :)."
	        << std::endl << "A\tEnable auto white balance" << std::endl << "Z\tDisable auto white balance" << std::endl
	        << "S\tIncrease exposure" << std::endl << "X\tDecrease exposure" << std::endl << "Q\tQuit program"
	        << std::endl << "Enter a key and press the \"Enter\" button" << std::endl;

	node.run();
	return 0;
}
