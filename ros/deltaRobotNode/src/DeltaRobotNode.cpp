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

#include "DeltaRobotNode/deltaRobotNode.h"
#include "DeltaRobotNode/Point.h"
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
deltaRobotNodeNamespace::DeltaRobotNode::DeltaRobotNode(int equipletID, int moduleID) : 
	rosMast::StateMachine(equipletID, moduleID),
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
	
	ros::NodeHandle nodeHandle;

	// Advertise the old deprecated services
	moveToPointService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_old, this);
	movePathService_old =	nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::movePath_old, this);
	moveToRelativePointService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_old, this);
	moveRelativePathService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_RELATIVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_old, this);
	calibrateService_old = nodeHandle.advertiseService(DeltaRobotNodeServices::CALIBRATE, &deltaRobotNodeNamespace::DeltaRobotNode::calibrate_old, this);

	// Advertise the json services
	moveToPointService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_POINT_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_json, this);
	movePathService_json =	nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_PATH_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::movePath_json, this);
	moveToRelativePointService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_json, this);
	moveRelativePathService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_RELATIVE_PATH_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_json, this);
	calibrateService_json = nodeHandle.advertiseService(DeltaRobotNodeServices::CALIBRATE_JSON, &deltaRobotNodeNamespace::DeltaRobotNode::calibrate_json, this);

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

	DataTypes::DeltaRobotMeasures drm;
	drm.base = DeltaRobot::Measures::BASE;
	drm.hip = DeltaRobot::Measures::HIP;
	drm.effector = DeltaRobot::Measures::EFFECTOR;
	drm.ankle = DeltaRobot::Measures::ANKLE;
	drm.maxAngleHipAnkle = DeltaRobot::Measures::HIP_ANKLE_ANGLE_MAX;

	modbus = new ModbusController::ModbusController(modbus_new_rtu(
		"/dev/ttyS0",
		Motor::CRD514KD::RtuConfig::BAUDRATE,
		Motor::CRD514KD::RtuConfig::PARITY,
		Motor::CRD514KD::RtuConfig::DATA_BITS,
		Motor::CRD514KD::RtuConfig::STOP_BITS));

	// Motors is declared in the header file, size = 3
	motors[0] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_0, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
	motors[1] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_1, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
	motors[2] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_2, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);

	motorManager = new Motor::MotorManager(modbus, motors, 3);

	// Create a deltarobot
	deltaRobot = new DeltaRobot::DeltaRobot(drm, motorManager, motors, modbusIO);
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
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate_old(deltaRobotNode::Calibrate::Request &req, deltaRobotNode::Calibrate::Response &res){
	if(getState() != rosMast::normal){
		res.succeeded = false;
		res.message = "Cannot calibrate, mast state=" + std::string(rosMast::state_txt[getState()]);
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
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate_json(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res){
	if(getState() != rosMast::normal){
		res.succeeded = false;
		res.message = "Cannot calibrate, mast state=" + std::string(rosMast::state_txt[getState()]);
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
	DataTypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
	DataTypes::Point3D<double> newLocation(x,y,z);

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
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_old(deltaRobotNode::MoveToPoint::Request &req, deltaRobotNode::MoveToPoint::Response &res) {
	ROS_INFO("moveToPoint_old called");
	
	if(getState() != rosMast::normal) {
		res.succeeded = false;
		res.message="Cannot move to point, mast state="+ std::string(rosMast::state_txt[getState()]);
	} else {
		res.succeeded = moveToPoint(req.motion.x, req.motion.y, req.motion.z, req.motion.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to point, path is illegal";
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
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint_json(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res){
	ROS_INFO("moveToPoint_json called");
	
	if(getState() != rosMast::normal){
		res.succeeded = false;
		res.message = "Cannot move to point, mast state=" + std::string(rosMast::state_txt[getState()]);
	} else {
		Point p = parsePoint(req.json);
		res.succeeded = moveToPoint(p.x, p.y, p.z, p.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to point, path is illegal";
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
	DataTypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
	DataTypes::Point3D<double> newLocation(x,y,z);
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
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_old(deltaRobotNode::MoveToRelativePoint::Request &req, deltaRobotNode::MoveToRelativePoint::Response &res){
	ROS_INFO("moveToRelativePoint_old called");
	
	if(getState() != rosMast::normal){
		res.succeeded = false;
		res.message = "Cannot move to relative point, mast state=" + std::string(rosMast::state_txt[getState()]);
	} else {
		res.succeeded = moveToPoint(req.motion.x, req.motion.y, req.motion.z, req.motion.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to relative point, path is illegal";
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
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint_json(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res){
	ROS_INFO("moveToRelativePoint_json called");
	
	if(getState() != rosMast::normal){
		res.succeeded = false;
		res.message = "Cannot move to relative point, mast state=" + std::string(rosMast::state_txt[getState()]);
	} else {
		Point p = parsePoint(req.json);
		res.succeeded = moveToPoint(p.x, p.y, p.z, p.maxAcceleration);

		if(!res.succeeded){
			res.message = "Cannot move to relative point, path is illegal";
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
bool deltaRobotNodeNamespace::DeltaRobotNode::movePath_old(deltaRobotNode::MovePath::Request &req, deltaRobotNode::MovePath::Response &res) {
	ROS_INFO("movePath_old called");
	res.succeeded = false;
	if(getState() != rosMast::normal) {
		res.message="Cannot move path, mast state="+ std::string(rosMast::state_txt[getState()]);
	} else {
		if(!deltaRobot->checkPath(deltaRobot->getEffectorLocation(),DataTypes::Point3D<double>(req.motion[0].x, req.motion[0].y, req.motion[0].z))){
			res.message = "Cannot move path, path is illegal";
			return true;
		}
		
		for(unsigned int i = 0; i < req.motion.size() -1; i++){
			if(!deltaRobot->checkPath(
					DataTypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z),
					DataTypes::Point3D<double>(req.motion[i+1].x, req.motion[i+1].y, req.motion[i+1].z))){
				res.message="Cannot move path, path is illegal";
				return true;
			}
		}
		
		for(unsigned int i = 0; i < req.motion.size(); i++){	
			ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", req.motion[i].x, req.motion[i].y, req.motion[i].z, req.motion[i].maxAcceleration);
			deltaRobot->moveTo(DataTypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z), req.motion[i].maxAcceleration);
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
bool deltaRobotNodeNamespace::DeltaRobotNode::movePath_json(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res){
	ROS_INFO("movePath_json called");
	
	res.succeeded = false;
	if(getState() != rosMast::normal){
		res.message = "Cannot move path, mast state=" + std::string(rosMast::state_txt[getState()]);
	} else {
		int size = 0;
		Point * path = parsePointArray(req.json, size);

		if(!deltaRobot->checkPath(deltaRobot->getEffectorLocation(),DataTypes::Point3D<double>(path[0].x, path[0].y, path[0].z))){
			res.message = "Cannot move path, path is illegal";
			return true;
		}

		for(int i = 0; i < (int)(size - 1); i++){
			if(!deltaRobot->checkPath(
					DataTypes::Point3D<double>(path[i].x, path[i].y, path[i].z),
					DataTypes::Point3D<double>(path[i + 1].x, path[i + 1].y, path[i + 1].z))){
				res.message = "Cannot move path, path is illegal";
				return true;
			}
		}

		// if the function gets to this point, the path is valid, we can move.
		for(int i = 0; i < (int)size; i++){
			ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", path[i].x, path[i].y, path[i].z, path[i].maxAcceleration);
			deltaRobot->moveTo(DataTypes::Point3D<double>(path[i].x, path[i].y, path[i].z), path[i].maxAcceleration);
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
bool deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_old(deltaRobotNode::MoveRelativePath::Request &req, deltaRobotNode::MoveRelativePath::Response &res) {
	ROS_INFO("moveRelativePath_old called");
    res.succeeded = false;
	if(getState() != rosMast::normal) {
		res.message = "Cannot move to relative path, mast state= " + std::string(rosMast::state_txt[getState()]);
	} else {
		DataTypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
		DataTypes::Point3D<double> newLocation(req.motion[0].x, req.motion[0].y, req.motion[0].z);
		newLocation += oldLocation;

		if(!deltaRobot->checkPath(oldLocation, newLocation)){
			res.message = "Cannot move relative path, path is illegal";
			ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed", oldLocation.x, oldLocation.y, oldLocation.z, newLocation.x, newLocation.y, newLocation.z);
			return true;
		}

		for(unsigned int i = 0; i < req.motion.size() - 1; i++){
			oldLocation += DataTypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z);
			newLocation += DataTypes::Point3D<double>(req.motion[i+1].x, req.motion[i+1].y, req.motion[i+1].z);

			if(!deltaRobot->checkPath(oldLocation, newLocation)){
				res.message="Cannot move relative path, path is illegal";
				ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed", oldLocation.x, oldLocation.y, oldLocation.z, newLocation.x, newLocation.y, newLocation.z);
				return true; 
			}
		}

		DataTypes::Point3D<double> currentLocation(deltaRobot->getEffectorLocation());
		for(unsigned int i = 0; i < req.motion.size(); i++){
			currentLocation += DataTypes::Point3D<double>(req.motion[i].x, req.motion[i].y, req.motion[i].z);
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
bool deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath_json(rexosStdSrvs::Module::Request &req, rexosStdSrvs::Module::Response &res){
	ROS_INFO("moveRelativePath_json called");
	res.succeeded = false;
	if(getState() != rosMast::normal){
		res.message = "Cannot move to relative path, mast state=" + std::string(rosMast::state_txt[getState()]);
	} else {
		int size = 0;
		Point * path = parsePointArray(req.json, size);

		DataTypes::Point3D<double> oldLocation(deltaRobot->getEffectorLocation());
		DataTypes::Point3D<double> newLocation(path[0].x, path[0].y, path[0].z);
		newLocation += oldLocation;
		
		if(deltaRobot->checkPath(oldLocation, newLocation)){
			res.message = "Cannot move path, path is illegal";
			return true;
		}
		
		for(int i = 0; i < (int)(size - 1); i++){
			oldLocation += DataTypes::Point3D<double>(path[i].x, path[i].y, path[i].z);
			newLocation += DataTypes::Point3D<double>(path[i+1].x, path[i+1].y, path[i+1].z);
			if(!deltaRobot->checkPath(oldLocation, newLocation)){
				res.message = "Cannot move to relative path, path is illegal";
				ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed", oldLocation.x, oldLocation.z, oldLocation.y, newLocation.x, newLocation.y, newLocation.z);
				return true;
			}
		}

		DataTypes::Point3D<double> currentLocation(deltaRobot->getEffectorLocation());
		for(int i = 0; i < (int)size; i++){
			currentLocation += DataTypes::Point3D<double>(path[i].x, path[i].y, path[i].z);
			ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", currentLocation.x, currentLocation.y, currentLocation.z, path[i].maxAcceleration);
			deltaRobot->moveTo(currentLocation, path[i].maxAcceleration);
		}
		res.succeeded = true;
		delete path;
	}
	return true;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
int deltaRobotNodeNamespace::DeltaRobotNode::transitionSetup(){
	ROS_INFO("Setup transition called");
	setState(rosMast::setup);

	// Generate the effector boundaries with voxel size 2
	deltaRobot->generateBoundaries(2);
	// Power on the deltarobot and calibrate the motors.
	deltaRobot->powerOn();
	// Calibrate the motors
	if(!deltaRobot->calibrateMotors()){
		ROS_ERROR("Calibration FAILED. EXITING.");
		return 1;
	}
	return 0;
}

/**
 * Transition from Standby to Safe state
 * Will turn power off the motor 
 * @return will be 0 if everything went ok else error
 **/
int deltaRobotNodeNamespace::DeltaRobotNode::transitionShutdown(){
	ROS_INFO("Shutdown transition called");
	setState(rosMast::shutdown);
	// Should have information about the workspace, calculate a safe spot and move towards it
	deltaRobot->powerOff();
	return 0;
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
int deltaRobotNodeNamespace::DeltaRobotNode::transitionStart(){
	ROS_INFO("Start transition called");
	// Set currentState to start
	setState(rosMast::start);
	// Could calibrate here
	return 0;
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
int deltaRobotNodeNamespace::DeltaRobotNode::transitionStop(){
	ROS_INFO("Stop transition called");
	// Set currentState to stop
	setState(rosMast::stop);
	// Go to base (Motors on 0 degrees)
	return 0;
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
			p.x = i->as_int();
		} else if(node_name == "y"){
			p.y = i->as_int();
		} else if(node_name == "z"){
			p.z = i->as_int();
		} else if(node_name == "maxAcceleration"){
			p.maxAcceleration = i->as_int();
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

	if(argc < 3 || !(Utilities::stringToInt(equipletID, argv[1]) == 0 && Utilities::stringToInt(moduleID, argv[2]) == 0)){ 	 	
		ROS_INFO("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -1;
	}

	ros::init(argc, argv, NODE_NAME);
	
	ROS_INFO("Creating DeltaRobotNode");

	deltaRobotNodeNamespace::DeltaRobotNode drn(equipletID, moduleID);

	ROS_INFO("Running StateEngine");
	ros::spin();
	return 0;
}
