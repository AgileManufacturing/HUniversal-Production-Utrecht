/**
 * @file DeltaRobotNode.cpp
 * @brief Provide the services to move the DeltaRobot.
 * @date Created: 2012-09-19
 *
 * @author Dick van der Steen
 * @author Dennis Koole
 * @authore Alexander Streng
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
#include <boost/bind.hpp>
#include <execinfo.h>
#include <signal.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "DeltaRobotNode"
// @endcond
/**
 * The IP of the modbus we are connecting to
 **/
#define MODBUS_IP "192.168.0.22"
/** 
 * The port we are connecting to
 **/
#define MODBUS_PORT 502

#define Z_OFFSET 0

/**
 * Constructor 
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the deltarobot
 **/
deltaRobotNodeNamespace::DeltaRobotNode::DeltaRobotNode(int equipletID, int moduleID) :
	rexos_statemachine::ModuleStateMachine("delta_robot_node",equipletID, moduleID, true),
	rexos_coordinates::Module(moduleID),
	deltaRobot(NULL),
	modbus(NULL),
	motorManager(NULL),
	setInstructionActionServer(nodeHandle, "delta_robot_node/set_instruction", boost::bind(&deltaRobotNodeNamespace::DeltaRobotNode::onSetInstruction, this, _1), false),
	lastX(0.0),
	lastY(0.0),
	lastZ(-180.0){
	ROS_INFO("DeltaRobotnode Constructor entering...");

	ROS_INFO("Configuring Modbus...");

	ROS_INFO("Advertising ActionServer at : delta_robot_node_1_1");


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

	setInstructionActionServer.start();
	ROS_INFO("DeltaRobot Node initialized");
}



deltaRobotNodeNamespace::DeltaRobotNode::~DeltaRobotNode() {
	delete deltaRobot;
	delete motors[0];
	delete motors[1];
	delete motors[2];
	delete modbus;
	delete motorManager;	
}


void deltaRobotNodeNamespace::DeltaRobotNode::onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal){
	JSONNode instructionDataNode = libjson::parse(goal->json);
	rexos_statemachine::SetInstructionResult result_;
	result_.OID = goal->OID;
	bool lookupIsSet = false;

    //construct a payload
    //construct lookupvalues.
	Point payloadPoint;
	double angle, normalLookupX, normalLookupY, rotatedLookUpX, rotatedLookupY;

    JSONNode::const_iterator i = instructionDataNode.begin();
    while (i != instructionDataNode.end()){
        const char * nodeName = i -> name().c_str();
	    // keep in mind that a payload may or may not contain all values. Use lastXYZ to determine these values if they are not set.
        if (strcmp(nodeName, "payload") == 0){
        	JSONNode payloadNode = *i;
			payloadPoint = parsePoint(payloadNode);
   			JSONNode::const_iterator j = payloadNode.begin();
		    while (j != payloadNode.end()) {

		    	const char * payloadNodeName = j -> name().c_str();

			    if (strcmp(payloadNodeName, "locationX") == 0){
					rotatedLookUpX = j->as_double();
			    	std::cout << "found locationX " << j->as_double() << " stringToDouble " << rotatedLookUpX << std::endl;
					lookupIsSet = true;
				}

			    if (strcmp(payloadNodeName, "locationY") == 0){
					rotatedLookupY = j->as_double();
			    	std::cout << "found locationY " << j->as_double() << " stringToDouble " << rotatedLookupY << std::endl;
					lookupIsSet = true;
				}

			    if (strcmp(payloadNodeName, "angle") == 0){
					angle = j->as_double();
			    	std::cout << "found angle " << j->as_double() << " stringToDouble " << angle << std::endl;
					lookupIsSet = true;
				}
			    j++;
		    }
        }
        ++i;
    }

    if(lookupIsSet) {
		double cs = cos(angle);
		double sn = sin(angle);
		normalLookupX = rotatedLookUpX * cs - rotatedLookupY * sn;
		normalLookupX = rotatedLookUpX * sn + rotatedLookupY * cs;
	}

    //translate the relative point to real equiplet coordinates.
	Vector3 lookupVector(normalLookupX, normalLookupY, 0);
	Vector3 translatedVector = convertToModuleCoordinate(lookupVector);

	std::cout << "translatedVector" << translatedVector << std::endl;

	if(moveToPoint((translatedVector.x + payloadPoint.x), (translatedVector.y + payloadPoint.y), payloadPoint.z, payloadPoint.maxAcceleration)){
		setInstructionActionServer.setSucceeded(result_);
		return;
	}

  	//finally move to point.
	ROS_INFO("Failed moving to point");
	setInstructionActionServer.setAborted(result_);
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

	ROS_INFO("moveTo: (%f, %f, %f) maxAcceleration=%f", x, y, z, maxAcceleration);

	if(deltaRobot->checkPath(oldLocation, newLocation)){

		deltaRobot->moveTo(newLocation, maxAcceleration);

		return true;
	}

	return false;
} 

/**
 * Function that moves the delta robot to a point that is relative to the current. 
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
		ROS_INFO("Moving to: (%f, %f, %f) maxAcceleration=%f", x, y, z, maxAcceleration);
		deltaRobot->moveTo(newLocation, maxAcceleration);
		return true;
	} else {
		return false;
	}
}


/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionSetup(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Setup transition called");

	// Generate the effector boundaries with voxel size 2
	deltaRobot->generateBoundaries(2);
	// Power on the deltarobot and calibrate the motors.
	deltaRobot->powerOn();
	// Calibrate the motors
	if(!deltaRobot->calibrateMotors()){
		ROS_ERROR("Calibration FAILED. EXITING.");
			as->setAborted();
	} else {
	as->setSucceeded();
	}
}

/**
 * Transition from Standby to Safe state
 * Will turn power off the motor 
 * @return will be 0 if everything went ok else error
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionShutdown(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	deltaRobot->powerOff();
	as->setSucceeded();
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionStart(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Start transition called");
	//The service servers should be set, to provide the normal methods for the equiplet
	as->setSucceeded();
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
void deltaRobotNodeNamespace::DeltaRobotNode::transitionStop(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO("Stop transition called");
	//The service servers should be set off, so the equiplet isn't able to set tasks for the module
		as->setSucceeded();
	// Go to base (Motors on 0 degrees)
}

/**
 * Parse a JSON string to a Point object
 *
 * @param json String that contains the json thats need to be parsed to a point
 *
 * @return Point object that is initialized from the data in the JSON
 **/
deltaRobotNodeNamespace::Point deltaRobotNodeNamespace::DeltaRobotNode::parsePoint(const JSONNode & n){

	JSONNode::const_iterator i = n.begin();
	Point p;

	bool xSet = false;
	bool ySet = false;

	ROS_INFO("parsing json to point");
	while(i != n.end()){
		// get the JSON node name and value as a string
		std::string node_name = i->name();

		if(node_name == "x") {
			p.x = i->as_float();
			lastX = p.x;
			xSet = true;
		} else if(node_name == "y") {
			p.y = i->as_float();
			lastY = p.y;
			ySet = true;
		} else if(node_name == "z") {
			p.z = i->as_float() + Z_OFFSET;
			lastZ = p.z;
		} else if(node_name == "maxAcceleration"){
			p.maxAcceleration = i->as_float();
		}

		++i;
	}

	if(p.x == 0 && !xSet){
		p.x = lastX;
	}
	if(p.y == 0 && !ySet){
		p.y = lastY;
	}
	if(p.z  == 0){
		p.z = lastZ;
	}
	
	return p;
}

deltaRobotNodeNamespace::Point deltaRobotNodeNamespace::DeltaRobotNode::parseLookup(const JSONNode & n){

	Point p;
	JSONNode::const_iterator i = n.begin();
	while(i != n.end()){
		// get the JSON node name and value as a string
		std::string node_name = i->name();

		if(node_name == "locationX") {
			p.x = i->as_float();
		} else if(node_name == "locationY") {
			p.y = i->as_float();
		} else if(node_name == "locationZ") {
			p.z = i->as_float();
		}
		++i;
	}
	return p;
}

std::string deltaRobotNodeNamespace::DeltaRobotNode::parseNodeValue(const std::string nodeName, const JSONNode & n){

	JSONNode::const_iterator i = n.begin();
	std::string result;
	while(i != n.end()){
		// get the JSON node name and value as a string
		std::string node_name = i->name();

		if(node_name == nodeName)
		{
			result = i->as_string();
		} 

		++i;
	}

	return result;
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
		Point p = parsePoint(*i);
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

	if (argc < 3) {
		ROS_INFO("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -1;
	}

	try{
		equipletID = rexos_utilities::stringToInt(argv[1]);
		moduleID = rexos_utilities::stringToInt(argv[2]);
	} catch(std::runtime_error ex) {
		ROS_ERROR("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -2;
	}


	ros::init(argc, argv, NODE_NAME);

	ROS_INFO("Creating DeltaRobotNode");

	deltaRobotNodeNamespace::DeltaRobotNode drn(equipletID, moduleID);

	ROS_INFO("Running StateEngine");
	ros::spin();
	return 0;
}
