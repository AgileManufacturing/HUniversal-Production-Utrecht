/**
 * @file DeltaRobotNode.cpp
 * @brief Provide the services to move the DeltaRobot.
 * @date Created: 2012-09-19
 *
 * @author Dick van der Steen
 * @author Dennis Koole
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

#include "ros/ros.h"
#include "deltaRobotNode/MoveToPoint.h"
#include "deltaRobotNode/MovePath.h"
#include "deltaRobotNode/MoveToRelativePoint.h"
#include "deltaRobotNode/MoveRelativePath.h"
#include "deltaRobotNode/Motion.h"
#include "deltaRobotNode/Calibrate.h"
#include "deltaRobotNode/Calibration.h"

#include <DataTypes/Point3D.h>
#include <DeltaRobot/DeltaRobot.h>
#include <Motor/StepperMotor.h>
#include <DeltaRobotNode/Services.h>

/**
 * @var NODE_NAME
 * Name for the DeltaRobotNode.
 **/
#define NODE_NAME "DeltaRobotNode"

/**
 * @var DeltaRobot * deltaRobot
 * Pointer to the DeltaRobot object.
 **/
static DeltaRobot::DeltaRobot * deltaRobot;

/**
 * Starts the (re)calibration of the robot.
 * 
 * @param request The request for this service as defined in Calibrate.srv
 * @param response The response for this service as defined in Calibrate.srv
 * 
 * @return True if the calibration was succesfully, false otherwise.
 **/
bool calibrate(deltaRobotNode::Calibrate::Request &request, deltaRobotNode::Calibrate::Response &response){
    // Calibrate the motors.
    if(!deltaRobot->calibrateMotors()){
    	ROS_ERROR("Calibration FAILED. EXITING.");
    	return false;
    }
	return true;	
}

/**
 * Move to an absolute point.
 *
 * @param request The request for this service as defined in MoveToPoint.srv
 * @param response The response for this service as defined in MoveToPoint.srv
 * 
 * @return True if the point has been reached, false if the point can't be reached or an error occurred during travelling.
 **/
bool moveToPoint(deltaRobotNode::MoveToPoint::Request &request, deltaRobotNode::MoveToPoint::Response &response){
	ROS_INFO("moveToPoint called");
	DataTypes::Point3D<double>& effectorLocation = deltaRobot->getEffectorLocation();
	deltaRobotNode::Motion motion = request.motion;
	
	/**
	 * Check if the DeltaRobot can move from the current effector location
	 * to the absolute point given as argument for this service.
	 **/
	if(!deltaRobot->checkPath(
		DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
		DataTypes::Point3D<double>(motion.x, motion.y, motion.z))){
		response.succeeded = false;
		return response.succeeded;
	}
	ROS_INFO("moveTo: (%f, %f, %f) speed=%f", motion.x, motion.y,motion.z, motion.speed);
	deltaRobot->moveTo(DataTypes::Point3D<double>(motion.x, motion.y, motion.z),motion.speed);
	response.succeeded = true;
	return response.succeeded;
}

/**
 * Move to a number of absolute points.
 *
 * @param request The request for this service as defined in MovePath.srv
 * @param response The response for this service as defined in MovePath.srv
 * 
 * @return True if the path is finished, false if path can't be travelled along or if an error occured during the travelling.
 **/
bool movePath(deltaRobotNode::MovePath::Request &request, deltaRobotNode::MovePath::Response &response){
	ROS_INFO("movePath called");
	deltaRobotNode::Motion currentMotion;
	deltaRobotNode::Motion nextMotion;
	try{
		unsigned int n;
		for(n = 0; n < request.motion.size() -1; n++){
			currentMotion = request.motion[n];			
			nextMotion = request.motion[n+1];
			if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(currentMotion.x, currentMotion.y, currentMotion.z),
				DataTypes::Point3D<double>(nextMotion.x, nextMotion.y, nextMotion.z))){
				response.succeeded = false;
				return response.succeeded;
			}
		}
		for(n = 0; n < request.motion.size(); n++){	
			currentMotion = request.motion[n];
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", currentMotion.x, currentMotion.y,currentMotion.z, currentMotion.speed);
			deltaRobot->moveTo(DataTypes::Point3D<double>(currentMotion.x, currentMotion.y, currentMotion.z),currentMotion.speed);
		}
	} catch(std::runtime_error& ex){
		std::stringstream ss;
		ss << "runtime error of type " << typeid(ex).name() << " in delta robot" << std::endl;
		ss << "what(): " << ex.what() << std::endl;
		response.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
		return response.succeeded;
	}
	response.succeeded = true;
	return response.succeeded;
}

/**
 * Move to a point that is relative to the current effector location.
 *
 * @param request The request for this service as defined in MoveToRelativePoint.srv
 * @param response The response for this service as defined in MoveToRelativePoint.srv
 * 
 * @return True if the point has been reached, false if the point can't be reached or an error occurred during travelling.
 **/
bool moveToRelativePoint(deltaRobotNode::MoveToRelativePoint::Request &request, deltaRobotNode::MoveToRelativePoint::Response &response){
	ROS_INFO("moveToRelativePoint called");
	deltaRobotNode::Motion currentMotion;
	try{
		currentMotion = request.motion;
		DataTypes::Point3D<double>& effectorLocation = deltaRobot->getEffectorLocation();
		ROS_INFO("Current effector location: x: %f y: %f z: %f", effectorLocation.x, effectorLocation.y, effectorLocation.z);
		double relativeX = effectorLocation.x + currentMotion.x;
		double relativeY = effectorLocation.y + currentMotion.y;
		double relativeZ = effectorLocation.z + currentMotion.z;
		ROS_INFO("Current motion z: %f", currentMotion.z);

		if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
				DataTypes::Point3D<double>(relativeX, relativeY, relativeZ))){
			response.succeeded = false;
			return response.succeeded;
		}
		deltaRobot->moveTo(DataTypes::Point3D<double>(relativeX, relativeY, relativeZ), currentMotion.speed);
		//deltaRobot->waitForReady();

	} catch(std::runtime_error& ex){
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		response.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}

    response.succeeded = true;
	return response.succeeded;
}

/**
 * Move to a number of relative points. 
 *
 * @param request The request for this service as defined in MoveRelativePath.srv
 * @param response The response for this service as defined in MoveRelativePath.srv
 *
 * @return True if the path is finished, false if path can't be travelled along or if an error occured during the travelling.
 **/
bool moveRelativePath(deltaRobotNode::MoveRelativePath::Request &request, deltaRobotNode::MoveRelativePath::Response &response){
	ROS_INFO("moveRelativePath called");

	deltaRobotNode::Motion currentMotion;
	double relativeX;
	double relativeY;
	double relativeZ;
	DataTypes::Point3D<double> effectorLocation;
	
	try{
		effectorLocation = deltaRobot->getEffectorLocation();
		unsigned int n;
		for(n = 0; n < request.motion.size(); n++){
			currentMotion = request.motion[n];			
			relativeX = effectorLocation.x + currentMotion.x;
			relativeY = effectorLocation.y + currentMotion.y;
			relativeZ = effectorLocation.z + currentMotion.z;
			if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
				DataTypes::Point3D<double>(relativeX, relativeY, relativeZ))){
				response.succeeded = false;
				ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed", effectorLocation.x, effectorLocation.z, effectorLocation.y, relativeX, relativeY, relativeZ);
				return response.succeeded;
			}
			effectorLocation.x = relativeX;
			effectorLocation.y = relativeY;
			effectorLocation.z = relativeZ;
		}
		for(n = 0; n < request.motion.size(); n++){	
			currentMotion = request.motion[n];			
			effectorLocation = deltaRobot->getEffectorLocation();
			relativeX = effectorLocation.x + currentMotion.x;
			relativeY = effectorLocation.y + currentMotion.y;
			relativeZ = effectorLocation.z + currentMotion.z;
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", relativeX, relativeY,relativeZ, currentMotion.speed);
			deltaRobot->moveTo(DataTypes::Point3D<double>(relativeX, relativeY, relativeZ),currentMotion.speed);
		}
	} catch(std::runtime_error& ex){
		std::stringstream ss;
		ss << "runtime error of type " << typeid(ex).name() << " in delta robot" << std::endl;
		ss << "what(): " << ex.what() << std::endl;
		response.succeeded = false;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
	}
	response.succeeded = true;
	return response.succeeded;
}

/**
 * Starting method for the DeltaRobotNode.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * 
 * @return 1 on calibration error.
 **/
int main(int argc, char** argv){
	ros::init(argc, argv, NODE_NAME);

    // Initialize modbus for IO controller
    modbus_t* modbusIO = modbus_new_tcp("192.168.0.2", 502);
    if(modbusIO == NULL){
        throw std::runtime_error("Unable to allocate libmodbus context");
    }
    if(modbus_connect(modbusIO) == -1){
        throw std::runtime_error("Modbus connection to IO controller failed");
    }
    assert(modbusIO != NULL);

    DataTypes::DeltaRobotMeasures deltaRobotMeasures;
    deltaRobotMeasures.base = DeltaRobot::Measures::BASE;
    deltaRobotMeasures.hip = DeltaRobot::Measures::HIP;
    deltaRobotMeasures.effector = DeltaRobot::Measures::EFFECTOR;
    deltaRobotMeasures.ankle = DeltaRobot::Measures::ANKLE;
    deltaRobotMeasures.maxAngleHipAnkle = DeltaRobot::Measures::HIP_ANKLE_ANGLE_MAX;

    ModbusController::ModbusController* modbus = new ModbusController::ModbusController(modbus_new_rtu(
        "/dev/ttyS0",
        Motor::CRD514KD::RtuConfig::BAUDRATE,
        Motor::CRD514KD::RtuConfig::PARITY,
        Motor::CRD514KD::RtuConfig::DATA_BITS,
        Motor::CRD514KD::RtuConfig::STOP_BITS));

    Motor::StepperMotor* motors[3];
    motors[0] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_0, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
    motors[1] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_1, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
    motors[2] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_2, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);

    Motor::MotorManager* motorManager = new Motor::MotorManager(modbus, motors, 3);

	// Create a deltarobot.	
    deltaRobot = new DeltaRobot::DeltaRobot(deltaRobotMeasures, motorManager, motors, modbusIO);
    // Generate the effector boundaries with voxel size 2.
    deltaRobot->generateBoundaries(2);
	// Power on the deltarobot.
    deltaRobot->powerOn();
    // Calibrate the motors.
    if(!deltaRobot->calibrateMotors()){
    	ROS_ERROR("Calibration FAILED. EXITING.");
    	return 1;
    }
    
	ros::NodeHandle nodeHandle;

	// Advertise the services.
	ros::ServiceServer moveToPointService = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_POINT, moveToPoint);
	ros::ServiceServer movePathService = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_PATH, movePath);
	ros::ServiceServer moveToRelativePointService = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT, moveToRelativePoint);
	ros::ServiceServer moveRelativePathService = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_RELATIVE_PATH, moveRelativePath);
	ros::ServiceServer calibrateService = nodeHandle.advertiseService(DeltaRobotNodeServices::CALIBRATE, calibrate);

	ROS_INFO("DeltaRobotNode ready...");

	ros::spin();
	return 0;
}
