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

#include "delta_robot_node/deltaRobotNode.h"
#include "delta_robot_node/Point.h"
#include <execinfo.h>
#include <signal.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "DeltaRobotNode"
// @endcond
/**
 * The IP of the modbus we are connecting to
 **/
#define MODBUS_IP "192.168.0.2"
/** 
 * The port we are connecting to
 **/
#define MODBUS_PORT 502

/**
 * Constructor 
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the deltarobot
 **/
deltaRobotNodeNamespace::DeltaRobotNode::DeltaRobotNode(int moduleID) :
	rexos_statemachine::StateMachine("delta_robot_node_" + moduleID),
	deltaRobot(NULL),
	modbus(NULL),
	motorManager(NULL),
	moveToPointService_old(),
	movePathService_old(),
	moveToRelativePointService_old(),
	moveRelativePathService_old(),
	calibrateService_old(),
	moveToPointService_json(),
	movePathService_json(),
	moveToRelativePointService_json(),
	moveRelativePathService_json(),
	calibrateService_json(){
	ROS_INFO("DeltaRobotnode Constructor entering...");

	ROS_INFO("Configuring Modbus...");

	// Initialize modbus for IO controller
	modbus_t *modbusIO = modbus_new_tcp(MODBUS_IP, MODBUS_PORT);
	if(modbusIO == NULL){
		throw std::runtime_error("Unable to allocate libmodbus context");
	}
	if(modbus_connect(modbusIO) == -1) {
		throw std::runtime_error("Modbus connection to IO controller failed");
	} 
	assert(modbusIO != NULL);

	rexos_datatypes::DeltaRobotMeasures drm;
	drm.base = rexos_delta_robot::Measures::BASE;
	drm.hip = rexos_delta_robot::Measures::HIP;
	drm.effector = rexos_delta_robot::Measures::EFFECTOR;
	drm.ankle = rexos_delta_robot::Measures::ANKLE;
	drm.maxAngleHipAnkle = rexos_delta_robot::Measures::HIP_ANKLE_ANGLE_MAX;

	modbus = new rexos_modbus::ModbusController(modbus_new_rtu(
		"/dev/ttyS0",
		rexos_motor::CRD514KD::RtuConfig::BAUDRATE,
		rexos_motor::CRD514KD::RtuConfig::PARITY,
		rexos_motor::CRD514KD::RtuConfig::DATA_BITS,
		rexos_motor::CRD514KD::RtuConfig::STOP_BITS));

	// Motors is declared in the header file, size = 3
	motors[0] = new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_0, rexos_delta_robot::Measures::MOTOR_ROT_MIN, rexos_delta_robot::Measures::MOTOR_ROT_MAX);
	motors[1] = new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_1, rexos_delta_robot::Measures::MOTOR_ROT_MIN, rexos_delta_robot::Measures::MOTOR_ROT_MAX);
	motors[2] = new rexos_motor::StepperMotor(modbus, rexos_motor::CRD514KD::Slaves::MOTOR_2, rexos_delta_robot::Measures::MOTOR_ROT_MIN, rexos_delta_robot::Measures::MOTOR_ROT_MAX);

	motorManager = new rexos_motor::MotorManager(modbus, motors, 3);

	// Create a deltarobot
	deltaRobot = new rexos_delta_robot::DeltaRobot(drm, motorManager, motors, modbusIO);
}

deltaRobotNodeNamespace::DeltaRobotNode::~DeltaRobotNode(){
	delete deltaRobot;
	delete motors[0];
	delete motors[1];
	delete motors[2];
	delete modbus;
	delete motorManager;	
}

// Calibrate service functions ------------------------------------------------
/**
 * Main function for starting the (re)calibratiion of the robot. Is called from the service functions. 
 *
 * @return true if the calibration was successful else false 
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate(){
	if(!deltaRobot->calibrateMotors()){
		ROS_ERROR("Calibration FAILED. EXITING.");
		return false;
	}
	return true;
}

/**
 * Function for the old, deprecated calibration service. Starts the (re)calibration of the robot.
 * @param req The request for this service as defined in the rexosStd package
 * @param res The response for this service as defined in the rexosStd package
 * 
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate_old(delta_robot_node::Calibrate::Request &req, delta_robot_node::Calibrate::Response &res){
	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.succeeded = false;
		res.message = "Cannot calibrate, mast state=" + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		res.succeeded = calibrate();
	}
	return true;
}

/**
 * Function for the json calibration service. Starts the (re)calibration of the robot.
 * @param req The request for this service as defined in the rexosStd package
 * @param res The response for this service as defined in the rexosStd package
 * 
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate_json(rexos_std_srvs::Module::Request &req, rexos_std_srvs::Module::Response &res){
	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.succeeded = false;
		res.message = "Cannot calibrate, mast state=" + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		res.succeeded = calibrate();
	}
	return true;
}



// moveToPoint service functions ----------------------------------------------
/**
 * moveToPoint function, is called by the json service functions and the old deprecated service functions.
 *
 * @param x destination x-coordinate
 * @param y destination y-coordinate
 * @param z destination z-coordinate
 * @param maxAcceleration maximum acceleration
 * 
 * @return false if the path is illegal, true if the motion is executed succesfully.
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint(double x, double y, double z, double maxAcceleration){
	rexos_datatypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
	rexos_datatypes::Point3D<double> newLocation(x,y,z);

	if(deltaRobot->checkPath(oldLocation, newLocation)){
		ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", x, y, z, maxAcceleration);
		deltaRobot->moveTo(newLocation, maxAcceleration);
		return true;
	}
	return false;
} 

/**
 * Old, deprecated service for moving the delta robot to an absolute position.
 *
 * @param req The request for this service as defined in MoveToPoint.srv 
 * @param res The response for this service as defined in MoveToPoint.srv
 * 
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_old(delta_robot_node::MoveToPoint::Request &req, delta_robot_node::MoveToPoint::Response &res) {
	ROS_INFO("moveToPoint_old called");

	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.succeeded = false;
		res.message="Cannot move to point, mast state="+ std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		res.succeeded = moveToPoint(req.motion.x, req.motion.y, req.motion.z, req.motion.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to point, path is illegal";
			ROS_INFO("%s",res.message.c_str());
		}
	}
	return true;
}

/**
 * Json service for moving the delta robot to an absolute position.
 *
 * @param req The request for this service as defined in the rexosStdMsgs, consisting of a json string containing a Point object.
 * @param res The response for this service as defined in the rexosStdMsgs
 * 
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_json(rexos_std_srvs::Module::Request &req, rexos_std_srvs::Module::Response &res){
	ROS_INFO("moveToPoint_json called");

	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.succeeded = false;
		res.message = "Cannot move to point, most state=" + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		Point p = parsePoint(req.json);
		res.succeeded = moveToPoint(p.x, p.y, p.z, p.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to point, path is illegal";
			ROS_INFO("%s",res.message.c_str());
		}
	}
	return true;
}

// moveToRelativePoint service functions --------------------------------------

/**
 * Function that moves the delta robot to a point that is relative to the current. Is called by the json service functions and the old deprecated service functions.
 *
 * @param x destination relative x-coordinate
 * @param y destination relative y-coordinate
 * @param z destination relative z-coordinate
 * @param maxAcceleration maximum acceleration
 * 
 * @return false if the path is illegal, true if the motion is executed succesfully.
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint(double x, double y, double z, double maxAcceleration){
	rexos_datatypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
	rexos_datatypes::Point3D<double> newLocation(x,y,z);
	newLocation += oldLocation;

	if(deltaRobot->checkPath(oldLocation, newLocation)){
		ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", x, y, z, maxAcceleration);
		deltaRobot->moveTo(newLocation, maxAcceleration);
		return true;
	} else {
		return false;
	}
}

/**
 * Old, deprecated service that moves the delta robot to a point that is relative to the current
 *
 * @param req The request for this service as defined in MoveToRelativePoint.srv 
 * @param res The response for this service as defined in MoveToRelativePoint.srv
 * 
 * @return always returns true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_old(delta_robot_node::MoveToRelativePoint::Request &req, delta_robot_node::MoveToRelativePoint::Response &res){
	ROS_INFO("moveToRelativePoint_old called");

	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.succeeded = false;
		res.message = "Cannot move to relative point, mast state=" + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		res.succeeded = moveToPoint(req.motion.x, req.motion.y, req.motion.z, req.motion.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to relative point, path is illegal";
			ROS_INFO("%s",res.message.c_str());
		}
	}
	return true;
}

/**
 * Json service that moves the delta robot to a point that is relative to the current
 *
 * @param req The request for this service as defined in the rexosStd package, consisting of a json string containing a Point object.
 * @param res The response for this service as defined in the rexosStd package
 * 
 * @return true when the moveToRelativePoint was successful, else false
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_json(rexos_std_srvs::Module::Request &req, rexos_std_srvs::Module::Response &res){
	ROS_INFO("moveToRelativePoint_json called");

	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.succeeded = false;
		res.message = "Cannot move to relative point, mast state=" + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		Point p = parsePoint(req.json);
		res.succeeded = moveToPoint(p.x, p.y, p.z, p.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to relative point, path is illegal";
			ROS_INFO("%s",res.message.c_str());
		}
	}
	return true;
}

// movePath service functions -------------------------------------------------

/**
 * Old, deprecated service function for moving along a path of absolute points.
 *
 * @param req The request for this service as defined in MovePath.srv 
 * @param res The response for this service as defined in MovePath.srv
 * 
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::movePath_old(delta_robot_node::MovePath::Request &req, delta_robot_node::MovePath::Response &res) {
	ROS_INFO("movePath_old called");
	res.succeeded = false;
	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.message="Cannot move path, mast state="+ std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		if(!deltaRobot->checkPath(deltaRobot->getEffectorLocation(),rexos_datatypes::Point3D<double>(req.motion[0].x, req.motion[0].y, req.motion[0].z))){
			res.message = "Cannot move path, path is illegal";
			ROS_INFO("%s",res.message.c_str());
			return true;
		}

		for(unsigned int i = 0; i < req.motion.size() -1; i++){
			if(!deltaRobot->checkPath(
					rexos_datatypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z),
					rexos_datatypes::Point3D<double>(req.motion[i+1].x, req.motion[i+1].y, req.motion[i+1].z))){
				res.message="Cannot move path, path is illegal";
				ROS_INFO("%s",res.message.c_str());
				return true;
			}
		}

		for(unsigned int i = 0; i < req.motion.size(); i++){	
			ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", req.motion[i].x, req.motion[i].y, req.motion[i].z, req.motion[i].maxAcceleration);
			deltaRobot->moveTo(rexos_datatypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z), req.motion[i].maxAcceleration);
		}
		res.succeeded = true;
	}
	return true;
}

/**
 * Json service function for moving along a path of absolute points.
 *
 * @param req The request for this service as defined in the rexosStd package, consisting of a json string containing an array of Point objects.
 * @param res The response for this service as defined in the rexosStd package.
 * 
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::movePath_json(rexos_std_srvs::Module::Request &req, rexos_std_srvs::Module::Response &res){
	ROS_INFO("movePath_json called");

	res.succeeded = false;
	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.message = "Cannot move path, mast state=" + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		int size = 0;
		Point * path = parsePointArray(req.json, size);

		if(!deltaRobot->checkPath(deltaRobot->getEffectorLocation(),rexos_datatypes::Point3D<double>(path[0].x, path[0].y, path[0].z))){
			res.message = "Cannot move path, path is illegal";
			ROS_INFO("%s",res.message.c_str());
			return true;
		}

		for(int i = 0; i < (int)(size - 1); i++){
			if(!deltaRobot->checkPath(
					rexos_datatypes::Point3D<double>(path[i].x, path[i].y, path[i].z),
					rexos_datatypes::Point3D<double>(path[i + 1].x, path[i + 1].y, path[i + 1].z))){
				res.message = "Cannot move path, path is illegal";
				ROS_INFO("%s",res.message.c_str());
				return true;
			}
		}

		// if the function gets to this point, the path is valid, we can move.
		for(int i = 0; i < (int)size; i++){
			ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", path[i].x, path[i].y, path[i].z, path[i].maxAcceleration);
			deltaRobot->moveTo(rexos_datatypes::Point3D<double>(path[i].x, path[i].y, path[i].z), path[i].maxAcceleration);
		}
		res.succeeded = true;
		delete path;
	}
	return true;
}

/**
 * Old, deprecated service function for moving along a path of points relative to eachother, the first one being relative to the current effector location.
 *
 * @param req The request for this service as defined in MoveRelativePath.srv
 * @param res The response for this service as defined in MoveRelativePath.srv
 *
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_old(delta_robot_node::MoveRelativePath::Request &req, delta_robot_node::MoveRelativePath::Response &res) {
	ROS_INFO("moveRelativePath_old called");
    res.succeeded = false;
    if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.message = "Cannot move to relative path, mast state= " + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		rexos_datatypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
		rexos_datatypes::Point3D<double> newLocation(req.motion[0].x, req.motion[0].y, req.motion[0].z);
		newLocation += oldLocation;

		if(!deltaRobot->checkPath(oldLocation, newLocation)){
			res.message = "Cannot move relative path, path is illegal";
			ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed", oldLocation.x, oldLocation.y, oldLocation.z, newLocation.x, newLocation.y, newLocation.z);
			return true;
		}

		for(unsigned int i = 0; i < req.motion.size() - 1; i++){
			oldLocation += rexos_datatypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z);
			newLocation += rexos_datatypes::Point3D<double>(req.motion[i+1].x, req.motion[i+1].y, req.motion[i+1].z);

			if(!deltaRobot->checkPath(oldLocation, newLocation)){
				res.message="Cannot move relative path, path is illegal";
				ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed", oldLocation.x, oldLocation.y, oldLocation.z, newLocation.x, newLocation.y, newLocation.z);
				return true; 
			}
		}

		rexos_datatypes::Point3D<double> currentLocation(deltaRobot->getEffectorLocation());
		for(unsigned int i = 0; i < req.motion.size(); i++){
			currentLocation += rexos_datatypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z);
			ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", currentLocation.x, currentLocation.y, currentLocation.z, req.motion[i].maxAcceleration);
			deltaRobot->moveTo(currentLocation, req.motion[i].maxAcceleration);
		}
		res.succeeded = true;
	}
	return true;
}

/**
 * Json service function for moving along a path of points relative to eachother, the first one being relative to the current effector location.
 *
 * @param req The request for this service as defined in the rexosStd package, consisting of a json string containing an array of Point objects.
 * @param res The response for this service as defined in the rexosStd package
 *
 * @return always true
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_json(rexos_std_srvs::Module::Request &req, rexos_std_srvs::Module::Response &res){
	ROS_INFO("moveRelativePath_json called");
	res.succeeded = false;
	if(getCurrentState() != rexos_statemachine::STATE_NORMAL){
		res.message = "Cannot move to relative path, mast state=" + std::string(rexos_statemachine::state_txt[getCurrentState()]);
		ROS_INFO("%s",res.message.c_str());
	} else {
		int size = 0;
		Point * path = parsePointArray(req.json, size);

		rexos_datatypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
		rexos_datatypes::Point3D<double> newLocation(path[0].x, path[0].y, path[0].z);
		newLocation += oldLocation;

		if(deltaRobot->checkPath(oldLocation, newLocation)){
			res.message = "Cannot move path, path is illegal";
			ROS_INFO("%s",res.message.c_str());
			return true;
		}

		for(int i = 0; i < (int)(size - 1); i++){
			oldLocation += rexos_datatypes::Point3D<double>(path[i].x, path[i].y, path[i].z);
			newLocation += rexos_datatypes::Point3D<double>(path[i+1].x, path[i+1].y, path[i+1].z);
			if(!deltaRobot->checkPath(oldLocation, newLocation)){
				res.message = "Cannot move to relative path, path is illegal";
				ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed", oldLocation.x, oldLocation.z, oldLocation.y, newLocation.x, newLocation.y, newLocation.z);
				return true;
			}
		}

		rexos_datatypes::Point3D<double> currentLocation(deltaRobot->getEffectorLocation());
		for(int i = 0; i < (int)size; i++){
			currentLocation += rexos_datatypes::Point3D<double>(path[i].x, path[i].y, path[i].z);
			ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", currentLocation.x, currentLocation.y, currentLocation.z, path[i].maxAcceleration);
			deltaRobot->moveTo(currentLocation, path[i].maxAcceleration);
		}
		res.succeeded = true;
		delete path;
	}
	return true;
}

/**
 * Will start the service servers of the module to make it able to perform tasks.
 */
void deltaRobotNodeNamespace::DeltaRobotNode::startServices(){
	// Advertise the old deprecated services
	moveToPointService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_old, this);
	movePathService_old =	nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::movePath_old, this);
	moveToRelativePointService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_old, this);
	moveRelativePathService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_RELATIVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_old, this);
	//calibrateService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::CALIBRATE, &deltaRobotNodeNamespace::DeltaRobotNode::calibrate_old, this);

	// Advertise the json services
	moveToPointService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_POINT_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_json, this);
	movePathService_json =	nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_PATH_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::movePath_json, this);
	moveToRelativePointService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_json, this);
	moveRelativePathService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_RELATIVE_PATH_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_json, this);
	//calibrateService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::CALIBRATE_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::calibrate_json, this);
}

/**
 * Shutdown the service servers
 */
void deltaRobotNodeNamespace::DeltaRobotNode::stopServices(){
	moveToPointService_old.shutdown();
	movePathService_old.shutdown();
	moveToRelativePointService_old.shutdown();
	moveRelativePathService_old.shutdown();
	moveToPointService_json.shutdown();
	movePathService_json.shutdown();
	moveToRelativePointService_json.shutdown();
	moveRelativePathService_json.shutdown();
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionSetup(){
	ROS_INFO("Setup transition called");

	// Generate the effector boundaries with voxel size 2
	deltaRobot->generateBoundaries(2);
	// Power on the deltarobot and calibrate the motors.
	deltaRobot->powerOn();
	// Calibrate the motors
	if(!deltaRobot->calibrateMotors()){
		ROS_ERROR("Calibration FAILED. EXITING.");
	}
}

/**
 * Transition from Standby to Safe state
 * Will turn power off the motor 
 * @return will be 0 if everything went ok else error
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionShutdown(){
	ROS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	deltaRobot->powerOff();
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionStart(){
	ROS_INFO("Start transition called");
	//The service servers should be set, to provide the normal methods for the equiplet
	startServices();
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionStop(){
	ROS_INFO("Stop transition called");
	//The service servers should be set off, so the equiplet isn't able to set tasks for the module
	stopServices();
	// Go to base (Motors on 0 degrees)
}

/**
 * Parse a JSON string to a Point object
 *
 * @param json String that contains the json thats need to be parsed to a point
 *
 * @return Point object that is initialized from the data in the JSON
 **/
deltaRobotNodeNamespace::Point deltaRobotNodeNamespace::DeltaRobotNode::parsePoint(std::string json){
	ROS_INFO("Parsing JSON");
	JSONNode n = libjson::parse(json);
	JSONNode::const_iterator i = n.begin();
	Point p;
	while(i != n.end()){
		// get the JSON node name and value as a string
		std::string node_name = i->name();

		if(node_name == "x"){
			p.x = i->as_float();
		} else if(node_name == "y"){
			p.y = i->as_float();
		} else if(node_name == "z"){
			p.z = i->as_float();
		} else if(node_name == "maxAcceleration"){
			p.maxAcceleration = i->as_float();
		}

		++i;
	}
	return p;
}

/**
 * Parse a JSON string to a Point object
 *
 * @param json String that contains the json thats need to be parsed to a point
 * @param size Reference to an int size, caller of the function needs to know this
 *
 * @return Point object that is initialized from the data in the JSON
 **/
deltaRobotNodeNamespace::Point* deltaRobotNodeNamespace::DeltaRobotNode::parsePointArray(std::string json, int &size){
	ROS_INFO("Parsing JSON Array");
	JSONNode pathArray = libjson::parse(json);
	Point *path = new Point[pathArray.size()];
	int counter = 0;

	JSONNode::const_iterator i = pathArray.begin();
	while(i != pathArray.end()){
		Point p = parsePoint(i->write());
		path[counter++] = p;
		++i;
	}

	size = pathArray.size();
	ROS_INFO("The size of the array %d", size);
	for(int i = 0; i < (int)size; i++){
		ROS_INFO("z value of item %d in array on position %d", (int)path[0].z, size);
	}
	ROS_INFO("Done parsing JSON Array");
	return path;
}

/**
 * Main that creates the deltaRobotNode and starts the statemachine
 **/
int main(int argc, char **argv){
	int equipletID = 0;
	int moduleID = 0;

	if(argc < 3 || !(rexos_utilities::stringToInt(equipletID, argv[1]) == 0 && rexos_utilities::stringToInt(moduleID, argv[2]) == 0)){ 	 	
		ROS_INFO("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -1;
	}

	ros::init(argc, argv, NODE_NAME);

	ROS_INFO("Creating DeltaRobotNode");

	deltaRobotNodeNamespace::DeltaRobotNode drn(moduleID);

	ROS_INFO("Running StateEngine");
	ros::spin();
	return 0;
}
