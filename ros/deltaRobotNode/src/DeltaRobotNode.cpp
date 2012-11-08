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
deltaRobotNodeNamespace::DeltaRobotNode::DeltaRobotNode(int equipletID, int moduleID) : rosMast::StateMachine(equipletID, moduleID)
{	
	ROS_INFO("DeltaRobotnode Constructor entering..."); 
	
	ros::NodeHandle nodeHandle;

	// Advertise the services
	moveToPointService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint, this);

	movePathService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::movePath, this);

	moveToRelativePointService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_TO_RELATIVE_POINT, &deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint, this);

	moveRelativePathService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::MOVE_RELATIVE_PATH, &deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath, this);

	calibrateService =
		nodeHandle.advertiseService(DeltaRobotNodeServices::CALIBRATE, &deltaRobotNodeNamespace::DeltaRobotNode::calibrate, this); 

	ROS_INFO("Configuring Modbus..."); 	

	// Initialize modbus for IO controller
    modbus_t* modbusIO = modbus_new_tcp(MODBUS_IP, MODBUS_PORT);
    if(modbusIO == NULL) {
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

    //Motors is declared in the header file, size = 3
    motors[0] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_0, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
    motors[1] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_1, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);
    motors[2] = new Motor::StepperMotor(modbus, Motor::CRD514KD::Slaves::MOTOR_2, DeltaRobot::Measures::MOTOR_ROT_MIN, DeltaRobot::Measures::MOTOR_ROT_MAX);

    motorManager = new Motor::MotorManager(modbus, motors, 3);

	// Create a deltarobot	
    deltaRobot = new DeltaRobot::DeltaRobot(drm, motorManager, motors, modbusIO); 
}

deltaRobotNodeNamespace::DeltaRobotNode::~DeltaRobotNode() {
	delete motors[0];
	delete motors[1];
	delete motors[2];	
	delete deltaRobot;
	delete motorManager;
	delete modbus;
}

/**
 * Starts the (re)calibration of the robot
 * @param req The request for this service as defined in Calibrate.srv 
 * @param res The response for this service as defined in Calibrate.srv
 * 
 * @return true if the calibration was successful else false
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate(deltaRobotNode::Calibrate::Request &req,
	deltaRobotNode::Calibrate::Response &res) {
	res.succeeded = false;
	if(getState() != rosMast::normal) {
		res.message="Cannot calibrate, mast state=" + std::string(rosMast::state_txt[getState()]);
		return true;
	}
    // Calibrate the motors
    if(!deltaRobot->calibrateMotors()) {
    	ROS_ERROR("Calibration FAILED. EXITING.");
    	res.message="Calibration FAILED. EXITING.";
    }
    res.succeeded = true;
	return true;	
}

/**
 * Move to an absolute point. Will be implemented in a later release.
 *
 * @param req The request for this service as defined in MoveToPoint.srv 
 * @param res The response for this service as defined in MoveToPoint.srv
 * 
 * @return true when the moveToPoint was successful, else false
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint(deltaRobotNode::MoveToPoint::Request &req,
	deltaRobotNode::MoveToPoint::Response &res) {
	ROS_INFO("moveToPoint called");
	res.succeeded = false;
	if(getState() != rosMast::normal) {
		res.message="Cannot move to point, mast state="+ std::string(rosMast::state_txt[getState()]);
		return true;
	}

	DataTypes::Point3D<double>& effectorLocation = deltaRobot->getEffectorLocation();
	deltaRobotNode::Motion motion = req.motion;
	/**
	 * Check if the DeltaRobot can move from the current effector location
	 * to the absolute point given as argument for this service.
	 **/
	if(!deltaRobot->checkPath(
		DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
		DataTypes::Point3D<double>(motion.x, motion.y, motion.z)))
	{
		res.message="Cannot move to point, path is illegal";
		return true;
	}
	ROS_INFO("moveTo: (%f, %f, %f) speed=%f", motion.x, motion.y,motion.z, motion.speed);
	deltaRobot->moveTo(DataTypes::Point3D<double>(motion.x, motion.y, motion.z),motion.speed);
	res.succeeded = true;
	return true;
}

/**
 * Move to a number of absolute points.
 *
 * @param req The request for this service as defined in MovePath.srv 
 * @param res The response for this service as defined in MovePath.srv
 * 
 * @return true when movePath was successful else false
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::movePath(deltaRobotNode::MovePath::Request &req,
	deltaRobotNode::MovePath::Response &res) {
	ROS_INFO("movePath called");
	res.succeeded = false;
	if(getState() != rosMast::normal) {
		res.message="Cannot move path, mast state="+ std::string(rosMast::state_txt[getState()]);
		return true;
	}

	deltaRobotNode::Motion currentMotion;
	deltaRobotNode::Motion nextMotion;
	try
	{
		unsigned int n;
		for(n = 0; n < req.motion.size() -1; n++)
		{
			currentMotion = req.motion[n];			
			nextMotion = req.motion[n+1];
			if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(currentMotion.x, currentMotion.y, currentMotion.z),
				DataTypes::Point3D<double>(nextMotion.x, nextMotion.y, nextMotion.z)))
			{
				res.message="Cannot move path, path is illegal";
				return true;
			}
		}
		for(n = 0; n < req.motion.size(); n++)
		{	
			currentMotion = req.motion[n];
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", currentMotion.x, currentMotion.y,currentMotion.z, currentMotion.speed);
			deltaRobot->moveTo(DataTypes::Point3D<double>(currentMotion.x, currentMotion.y, currentMotion.z),currentMotion.speed);
		}
	}
	catch(std::runtime_error& ex)
	{
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
		return true;
	}
	res.succeeded = true;
	return res.succeeded;
}

/**
 * Move to a point that is relative to the current effector location
 *
 * @param req The request for this service as defined in MoveToRelativePoint.srv 
 * @param res The response for this service as defined in MoveToRelativePoint.srv
 * 
 * @return true when the moveToRelativePoint was successful, else false
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToRelativePoint(deltaRobotNode::MoveToRelativePoint::Request &req,
	deltaRobotNode::MoveToRelativePoint::Response &res) {
	ROS_INFO("moveToRelativePoint called");
	res.succeeded = false;
	if(getState() != rosMast::normal) {
		res.message="Cannot move to relative point, mast state="+ std::string(rosMast::state_txt[getState()]);
		return true;
	}

	deltaRobotNode::Motion currentMotion;
	try {
		currentMotion = req.motion;
		DataTypes::Point3D<double>& effectorLocation = deltaRobot->getEffectorLocation();
		ROS_INFO("Current effector location: x: %f y: %f z: %f", effectorLocation.x, effectorLocation.y, effectorLocation.z);
		double relativeX = effectorLocation.x + currentMotion.x;
		double relativeY = effectorLocation.y + currentMotion.y;
		double relativeZ = effectorLocation.z + currentMotion.z;
		ROS_INFO("Current motion z: %f", currentMotion.z);

		if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
				DataTypes::Point3D<double>(relativeX, relativeY, relativeZ)))
		{
			res.message="Cannot move to relative point, path is illegal";
			return true;
		}
		deltaRobot->moveTo(DataTypes::Point3D<double>(relativeX, relativeY, relativeZ), currentMotion.speed);

	} catch(std::runtime_error& ex) {
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
		return true;
	}

    res.succeeded = true;
	return res.succeeded;
}

/**
 * Move to a number of relative points. 
 *
 * @param req The request for this service as defined in MoveRelativePath.srv 
 * @param res The response for this service as defined in MoveRelativePath.srv
 *
 * @return true if path is allowed else return false.
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::moveRelativePath(deltaRobotNode::MoveRelativePath::Request &req,
	deltaRobotNode::MoveRelativePath::Response &res) {
	ROS_INFO("moveRelativePath called");
    res.succeeded = false;
	if(getState() != rosMast::normal) {
		res.message="Cannot move to relative path, mast state="+ std::string(rosMast::state_txt[getState()]);
		return true;
	}

	deltaRobotNode::Motion currentMotion;
	double relativeX;
	double relativeY;
	double relativeZ;
	DataTypes::Point3D<double> effectorLocation;
	try
	{
		effectorLocation = deltaRobot->getEffectorLocation();
		unsigned int n;
		for(n = 0; n < req.motion.size(); n++)
		{
			currentMotion = req.motion[n];			
			relativeX = effectorLocation.x + currentMotion.x;
			relativeY = effectorLocation.y + currentMotion.y;
			relativeZ = effectorLocation.z + currentMotion.z;
			if(!deltaRobot->checkPath(
				DataTypes::Point3D<double>(effectorLocation.x, effectorLocation.y, effectorLocation.z),
				DataTypes::Point3D<double>(relativeX, relativeY, relativeZ)))
			{
				res.message="Cannot move to relative path, path is illegal";
				ROS_INFO("FROM %f, %f, %f TO %f, %f, %f Not allowed",effectorLocation.x,effectorLocation.z,effectorLocation.y,relativeX,relativeY,relativeZ );
				return true;
			}
			effectorLocation.x = relativeX;
			effectorLocation.y = relativeY;
			effectorLocation.z = relativeZ;
		}
		for(n = 0; n < req.motion.size(); n++)
		{	
			currentMotion = req.motion[n];			
			effectorLocation = deltaRobot->getEffectorLocation();
			relativeX = effectorLocation.x + currentMotion.x;
			relativeY = effectorLocation.y + currentMotion.y;
			relativeZ = effectorLocation.z + currentMotion.z;
			ROS_INFO("moveTo: (%f, %f, %f) speed=%f", relativeX, relativeY,relativeZ, currentMotion.speed);
			deltaRobot->moveTo(DataTypes::Point3D<double>(relativeX, relativeY, relativeZ),currentMotion.speed);
		}
	}
	catch(std::runtime_error& ex)
	{
		std::stringstream ss;
		ss << "runtime error of type "<< typeid(ex).name()<<" in delta robot" << std::endl;
		ss <<"what(): " << ex.what()<<std::endl;
		ROS_ERROR("moveTo: %s", ss.str().c_str());
		return true;
	}
	res.succeeded = true;
	return res.succeeded;
}
/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
int deltaRobotNodeNamespace::DeltaRobotNode::transitionSetup() {
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
int deltaRobotNodeNamespace::DeltaRobotNode::transitionShutdown() {	
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
int deltaRobotNodeNamespace::DeltaRobotNode::transitionStart() {
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
int deltaRobotNodeNamespace::DeltaRobotNode::transitionStop() {
	ROS_INFO("Stop transition called");	
	// Set currentState to stop
	setState(rosMast::stop);
	// Go to base (Motors on 0 degrees)
	return 0;
}

/** 
 * Main that creates the deltaRobotNode and starts the statemachine
 **/
int main(int argc, char **argv) {
	int equipletID = 0;
	int moduleID = 0;

	if(argc < 3 || !(Utilities::stringToInt(equipletID, argv[1]) == 0 && Utilities::stringToInt(moduleID, argv[2]) == 0))
	{ 	 	
    	ROS_INFO("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
    	return -1;
  	} 

	ros::init(argc, argv, NODE_NAME);
	
	ROS_INFO("Creating DeltaRobotNode"); 	

	deltaRobotNodeNamespace::DeltaRobotNode drn(equipletID, moduleID);    
	
	ROS_INFO("Running StateEngine"); 	
    drn.startStateMachine();	
	return 0;
}