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

#include "delta_robot_node/DeltaRobotNode.h"
#include "delta_robot_node/Point.h"
#include <boost/bind.hpp>
#include <execinfo.h>
#include <signal.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "DeltaRobotNode"
// @endcond

/**
 * Constructor 
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the deltarobot
 **/
deltaRobotNodeNamespace::DeltaRobotNode::DeltaRobotNode(int equipletID, int moduleID, std::string manufacturer, std::string typeNumber, std::string serialNumber) :
		rexos_knowledge_database::Module(manufacturer, typeNumber, serialNumber),
		rexos_statemachine::ModuleStateMachine("delta_robot_node", equipletID, moduleID, true),
		rexos_coordinates::Module(this),
		deltaRobot(NULL),
		moduleNodeName("delta_robot_node_" + std::to_string(equipletID) + "_" + std::to_string(moduleID)),
		setInstructionActionServer(nodeHandle, moduleNodeName + "/set_instruction", boost::bind(&deltaRobotNodeNamespace::DeltaRobotNode::onSetInstruction, this, _1), false),
		lastX(0.0),
		lastY(0.0),
		lastZ(0.0){
	ROS_INFO("DeltaRobotnode Constructor entering...");
	// get the properties and combine them for the deltarobot
	rexos_knowledge_database::ModuleType* moduleType = this->getModuleType();
	std::string properties = this->getModuleProperties();
	std::string typeProperties = moduleType->getModuleTypeProperties();


	JSONNode jsonNode = libjson::parse(properties);
	JSONNode typeJsonNode = libjson::parse(typeProperties);

	for(JSONNode::const_iterator it = typeJsonNode.begin(); it != typeJsonNode.end(); it++) {
		jsonNode.push_back(*it);
	}

	ROS_INFO("%s", jsonNode.write_formatted().c_str());
	
	// Create a deltarobot
	deltaRobot = new rexos_delta_robot::DeltaRobot(jsonNode);

	setInstructionActionServer.start();

	ROS_INFO_STREAM("DeltaRobotNode initialized. Advertising actionserver on " << moduleNodeName << "/set_instruction");
}



deltaRobotNodeNamespace::DeltaRobotNode::~DeltaRobotNode() {
	delete deltaRobot;
}


void deltaRobotNodeNamespace::DeltaRobotNode::onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal){
	JSONNode instructionDataNode = libjson::parse(goal->json);
	rexos_statemachine::SetInstructionResult result_;
	result_.OID = goal->OID;
	bool lookupIsSet = false;
	bool movementZ = false;
	std::string setValues;

    //construct a payload
    //construct lookupvalues.
	Point payloadPoint, lookupResultPoint;
	double angle, rotatedX, rotatedY;
    JSONNode::const_iterator i = instructionDataNode.begin();
    while (i != instructionDataNode.end()){
        const char * nodeName = i -> name().c_str();
	    // keep in mind that a payload may or may not contain all values. Use lastXYZ to determine these values if they are not set.
        if (strcmp(nodeName, "payload") == 0){

			payloadPoint = parsePoint(*i, &setValues);
			lookupResultPoint = parseLookup(*i);
			std::string angleValue = parseNodeValue("angle", *i);
			angle = rexos_utilities::stringToDouble(angleValue);

			//check whether lookup is set. If all values are 0, we can presume the lookup isnt set.
			//Bit dangerous tho, what happends if they are all exactly 0?
			if(!(lookupResultPoint.x == 0 && lookupResultPoint.y == 0 && angle == 0)){
				lookupIsSet = true;
			}

			if(setValues.find("x") == -1 && setValues.find("y") == -1) {
				// Probably a Z movement
				payloadPoint.x = deltaRobot->getEffectorLocation().x;
				payloadPoint.y = deltaRobot->getEffectorLocation().y;
				movementZ = true;
			}

			if(setValues.find("z") == -1) {
				// Probably a XY movement
				payloadPoint.z = deltaRobot->getEffectorLocation().z;
			}

        }
        ++i;
    }

    Vector3 moveVector;
    //lookup is set, so transform the (rotated) crate to a normal position.
    if(lookupIsSet) {
		Vector3 lookupVector(lookupResultPoint.x, lookupResultPoint.y, lookupResultPoint.z);

		double theta = angle * 3.141592653589793 / 180.0;
		double cs = cos(theta);
		double sn = sin(theta);
		rotatedX = payloadPoint.x * cs - payloadPoint.y * sn;
		rotatedY = payloadPoint.x * sn + payloadPoint.y * cs;

	    //translate the relative point to real equiplet coordinates.
		Vector3 translatedVector = convertToModuleCoordinate(lookupVector);

		std::cout << "[TranslatedVectorX]: \t" << translatedVector.x << "\n[TranslatedVectorY]: \t" << translatedVector.y << std::endl;

		if(movementZ) {
			// Z Movement
			moveVector.set(payloadPoint.x, payloadPoint.y, (translatedVector.z + payloadPoint.z));
		} else {
			// XY Movement
			moveVector.set((translatedVector.x + rotatedX), (translatedVector.y + rotatedY), payloadPoint.z);
			std::cout << "[MoveVectorX]: \t" << moveVector.x << "\n[MoveVectorY]: \t" << moveVector.y << std::endl;
		}

	} else {
		moveVector.set(payloadPoint.x, payloadPoint.y, payloadPoint.z);
	}

	//std::cout << "trying to move to x: " << moveVector.x << " y: " << moveVector.y << " z: " <<  moveVector.z << " with acceleration: " << payloadPoint.maxAcceleration << std::endl;
	if(moveToPoint(moveVector.x, moveVector.y, moveVector.z, payloadPoint.maxAcceleration)){
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
deltaRobotNodeNamespace::Point deltaRobotNodeNamespace::DeltaRobotNode::parsePoint(const JSONNode & n, std::string * valuesSet){

	JSONNode::const_iterator i = n.begin();
	Point p;

	while(i != n.end()){
		// get the JSON node name and value as a string
		std::string node_name = i->name();

		if(node_name == "x") {
			p.x = i->as_float();
			*valuesSet += "x";
		} else if(node_name == "y") {
			p.y = i->as_float();
			*valuesSet += "y";
		} else if(node_name == "z") {
			p.z = i->as_float();
			*valuesSet += "z";
		} else if(node_name == "maxAcceleration"){
			p.maxAcceleration = i->as_float();
		}

		++i;
	}
	
	return p;
}

deltaRobotNodeNamespace::Point deltaRobotNodeNamespace::DeltaRobotNode::parseLookup(const JSONNode & n){

	Point p;
	JSONNode::const_iterator i = n.begin();
	while(i != n.end()) {
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
	std::string discarded;
	JSONNode pathArray = libjson::parse(json);
	Point *path = new Point[pathArray.size()];
	int counter = 0;

	JSONNode::const_iterator i = pathArray.begin();
	while(i != pathArray.end()){
		Point p = parsePoint(*i, &discarded);
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
	ros::init(argc, argv, NODE_NAME);
	
	if(argc < 6){
		ROS_ERROR("Usage: delta_robot_node equipletId, moduleId, manufacturer, typeNumber, serialNumber");
		return -1;
	}
	
	
	int equipletID;
	int moduleID;
	try{
		equipletID = rexos_utilities::stringToInt(argv[1]);
		moduleID = rexos_utilities::stringToInt(argv[2]);
	} catch(std::runtime_error ex) {
		ROS_ERROR("Cannot read equiplet id and/or moduleId from commandline please use correct values.");
		return -2;
	}
	
	ROS_INFO("Creating DeltaRobotNode");

	deltaRobotNodeNamespace::DeltaRobotNode drn(equipletID, moduleID, argv[3], argv[4], argv[5]);

	ROS_INFO("Running StateEngine");
	ros::spin();
	return 0;
}
