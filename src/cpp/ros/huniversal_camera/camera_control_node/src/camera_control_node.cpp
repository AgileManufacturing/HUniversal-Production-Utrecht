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


#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
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
	std::cout << "Calling service increaseExposureClient " << printResult(increaseExposureClient.call(emptyService))
	        << std::endl;
}

void CameraControlNode::decreaseExposureCall() {
	std::cout << "Calling service decreaseExposureClient " << printResult(decreaseExposureClient.call(emptyService))
	        << std::endl;
}

void CameraControlNode::autoWhiteBalanceCall(bool enabled) {
	camera_node::AutoWhiteBalance awb;
	awb.request.enable = enabled;
	std::cout << "Calling service autoWhiteBalanceClient " << enabled << " " << printResult(autoWhiteBalanceClient.call(awb))
	        << std::endl;
}

inline std::string CameraControlNode::printResult(bool result) {
	return (result ? "[OK]" : "[FAIL]");
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
void CameraControlNode::transitionSetup(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Setup transition called");
	
	sql::Driver* driver = get_driver_instance();
	/* Create a connection */
	std::auto_ptr<sql::Connection> apConnection(
		driver->connect("tcp://192.168.65.175:3306", "rexos", "rexos")
	);
	/* Connect to the MySQL test database */
	apConnection->setSchema("rexos");
	
	sql::PreparedStatement* pstmt = apConnection->prepareStatement("SELECT * FROM calibrationmatrices WHERE camera = ? AND lens = ?");
	pstmt->setInt(1, cameraModuleId);
	pstmt->setInt(2, lensModuleId);
	
	sql::ResultSet* res = pstmt->executeQuery();
	
	if(res->rowsCount() == 1){
		// we have found a calibrateLens correction matrix
		ROS_INFO("calibrateLens correction matrix found");
		
		ros::ServiceClient client = nodeHandle.serviceClient<camera_node::setCorrectionMatrices>(camera_node_services::SET_CORRECTION_MATRICES);
		camera_node::setCorrectionMatrices serviceCall;
		
		// set the cursor at the first result
		res->next(); 
		
		// yeah, this is quite ugly
		serviceCall.request.distCoeffs.push_back(res->getDouble("distCoef_0"));
		serviceCall.request.distCoeffs.push_back(res->getDouble("distCoef_1"));
		serviceCall.request.distCoeffs.push_back(res->getDouble("distCoef_2"));
		serviceCall.request.distCoeffs.push_back(res->getDouble("distCoef_3"));
		serviceCall.request.distCoeffs.push_back(res->getDouble("distCoef_4"));
	
		serviceCall.request.cameraMatrix.values[0] = res->getDouble("cameraMatrix_0_0");
		serviceCall.request.cameraMatrix.values[1] = res->getDouble("cameraMatrix_0_1");
		serviceCall.request.cameraMatrix.values[2] = res->getDouble("cameraMatrix_0_2");
		serviceCall.request.cameraMatrix.values[3] = res->getDouble("cameraMatrix_1_0");
		serviceCall.request.cameraMatrix.values[4] = res->getDouble("cameraMatrix_1_1");
		serviceCall.request.cameraMatrix.values[5] = res->getDouble("cameraMatrix_1_2");
		serviceCall.request.cameraMatrix.values[6] = res->getDouble("cameraMatrix_2_0");
		serviceCall.request.cameraMatrix.values[7] = res->getDouble("cameraMatrix_2_1");
		serviceCall.request.cameraMatrix.values[8] = res->getDouble("cameraMatrix_2_2");
		
		client.call(serviceCall);
		
		as->setSucceeded();
	}
	else{
		camera_node::fishEyeCorrection serviceCall;
		serviceCall.request.enable = false;
		fishEyeCorrectionClient.call(serviceCall);

		camera_calibration_node::calibrateLens serviceCall2;
		serviceCall2.request.frameCount = 20;
		serviceCall2.request.boardWidth = 9;
		serviceCall2.request.boardHeight = 6;
		calibrateLensClient.call(serviceCall2);
		
		camera_node::fishEyeCorrection serviceCall3;
		serviceCall3.request.enable = true;
		fishEyeCorrectionClient.call(serviceCall3);
		
		if(serviceCall2.response.processedFrames == 0){
			ROS_WARN_STREAM("calibrateLens failed, processedFrame = " << serviceCall2.response.processedFrames);
			as->setAborted();
		}
		else{
			ROS_INFO_STREAM("Saving matrices to database for cameraId=" << cameraModuleId << " and lensModuleId=" << lensModuleId);
			camera_node::getCorrectionMatrices serviceCall4;
			getCorrectionMatricesClient.call(serviceCall4);
			
			sql::PreparedStatement* pstmt2 = apConnection->prepareStatement(
			"INSERT INTO calibrationmatrices ( \
				camera, lens, \
				distCoef_0,  distCoef_1,  distCoef_2, distCoef_3,  distCoef_4, \
				cameraMatrix_0_0, cameraMatrix_0_1, cameraMatrix_0_2, \
				cameraMatrix_1_0, cameraMatrix_1_1, cameraMatrix_1_2, \
				cameraMatrix_2_0, cameraMatrix_2_1, cameraMatrix_2_2 \
			) VALUES ( \
				?, ?, \
				?, ?, ?, ?, ?, \
				?, ?, ?, \
				?, ?, ?, \
				?, ?, ? \
			);");
			pstmt2->setInt(1, cameraModuleId);
			pstmt2->setInt(2, lensModuleId);
			
			int collumIndex = 3;
			for(int i = 0; i < serviceCall4.response.distCoeffs.size(); i++) {
				pstmt2->setDouble(collumIndex, serviceCall4.response.distCoeffs[i]);
				collumIndex++;
			}
			
			for(int i = 0; i < 9; i++) {
				pstmt2->setDouble(collumIndex, serviceCall4.response.cameraMatrix.values[i]);
				collumIndex++;
			}
			
			pstmt2->executeQuery();
			
			
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
