/**
 * @file StewartGoughNode.cpp
 * @brief Provide the services to move the Stewart Gough robot.
 * @date Created: 2014-02-19
 *
 * @author Garik Hakopian
 * @author Rolf Smit
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


#include "stewart_gough_node/StewartGoughNode.h"
#include "stewart_gough_node/Point.h"
#include <execinfo.h>
#include <signal.h>

#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "StewartGoughNode"
// @endcond

/**
 * Constructor 
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the deltarobot
 **/
stewartGoughNodeNamespace::StewartGoughNode::StewartGoughNode(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier) :
		rexos_knowledge_database::Module(moduleIdentifier),
		rexos_statemachine::ModuleStateMachine(equipletName, moduleIdentifier, true),
		rexos_coordinates::Module(this),
		stewartGough(NULL),
		setInstructionActionServer(
				nodeHandle, 
				equipletName + "/" + moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber() + "/set_instruction", 
				boost::bind(&stewartGoughNodeNamespace::StewartGoughNode::onSetInstruction, this, _1), 
				false),
		lastX(0.0),
		lastY(0.0),
		lastZ(0.0){
	ROS_INFO("StewartGoughNode Constructor entering...");
	// get the properties and combine them for the deltarobot
	std::string properties = this->getModuleProperties();
	std::string typeProperties = this->getModuleTypeProperties();


	Json::Reader reader;
	Json::Value jsonNode;
	Json::Value typeJsonNode;
	reader.parse(properties, jsonNode);
	reader.parse(typeProperties, typeJsonNode);
	
	std::vector<std::string> typeJsonNodeMemberNames = typeJsonNode.getMemberNames();
	for(int i = 0; i < typeJsonNodeMemberNames.size(); i++) {
		jsonNode[typeJsonNodeMemberNames[i]] = typeJsonNode[typeJsonNodeMemberNames[i]];
	}
	
	Json::StyledWriter writer;
	ROS_INFO("%s", writer.write(jsonNode).c_str());
		
	// Create a stewart gough robot
	stewartGough = new rexos_stewart_gough::StewartGough(jsonNode);

	setInstructionActionServer.start();

	ROS_INFO_STREAM("StewartGoughNode initialized. Advertising actionserver on " << 
			moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber() << 
			"/set_instruction");
}



stewartGoughNodeNamespace::StewartGoughNode::~StewartGoughNode() {
	delete stewartGough;
}


void stewartGoughNodeNamespace::StewartGoughNode::onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal){
	Json::Reader reader;
	Json::Value instructionDataNode;
	reader.parse(goal->json, instructionDataNode);
	
	rexos_statemachine::SetInstructionResult result_;
	result_.OID = goal->OID;
	bool lookupIsSet = false;
	bool movementZ = false;
	std::string setValues;
	
    //construct a payload
    //construct lookupvalues.
	std::cout <<"onSetInstruction" << std::endl;
	Point payloadPoint, lookupResultPoint;
	
	double rotationX = 0, rotationY = 0, rotationZ = 0;
	
	double angle, rotatedX, rotatedY;
    /*JSONNode::const_iterator i = instructionDataNode.begin();
	
	std::cout << "Json data: " << goal->json << std::endl;
	
	
	
    /*while (i != instructionDataNode.end()){
        const char * nodeName = i -> name().c_str();
	    // keep in mind that a payload may or may not contain all values. Use lastXYZ to determine these values if they are not set.
        if (strcmp(nodeName, "payload") == 0){

			
			payloadPoint = parsePoint(*i, &setValues);
			lookupResultPoint = parseLookup(*i);
			std::string angleValue = parseNodeValue("angle", *i);
			angle = rexos_utilities::stringToDouble(angleValue);
			
			
			//std::string rotateString = parseNodeValue("rotate", *i);
			
			//std::cout << "Rotate value: " << rotateString << std::endl;
				
			//if(!rotateString.empty()){
				
				
				//JSONNode::const_iterator rotateIterator = libjson::parse(rotateString).begin();
				
				
				std::string rotationValueX = parseNodeValue("rotationX", *i);
				rotationX = rexos_utilities::stringToDouble(rotationValueX);
				if(rotationValueX.empty()){
					rotationX = stewartGough->getEffectorRotationX();
				}
				
				std::string rotationValueY = parseNodeValue("rotationY", *i);
				rotationY = rexos_utilities::stringToDouble(rotationValueY);
				if(rotationValueY.empty()){
					rotationY = stewartGough->getEffectorRotationY();
				}
				
				std::string rotationValueZ = parseNodeValue("rotationZ", *i);
				rotationZ = rexos_utilities::stringToDouble(rotationValueZ);
				if(rotationValueZ.empty()){
					rotationZ = stewartGough->getEffectorRotationZ();
				
				}
			
			
			std::cout << "rotationX: " << rotationX << std::endl;
			std::cout << "rotationY: " << rotationY << std::endl;
			std::cout << "rotationZ: " << rotationZ << std::endl;
			
			
			//check whether lookup is set. If all values are 0, we can presume the lookup isnt set.
			//Bit dangerous tho, what happends if they are all exactly 0?
			if(!(lookupResultPoint.x == 0 && lookupResultPoint.y == 0 && angle == 0)){
				lookupIsSet = true;
			}
			

			if(setValues.find("x") == -1) {
				payloadPoint.x = stewartGough->getEffectorLocation().x;
			}
			if(setValues.find("y") == -1) {
				payloadPoint.y = stewartGough->getEffectorLocation().y;
			}
			if(setValues.find("z") == -1) {
				payloadPoint.z = stewartGough->getEffectorLocation().z;
			}
			
			
			/*
			if(setValues.find("x") == -1 && setValues.find("y") == -1) {
				// Probably a Z movement
				payloadPoint.x = stewartGough->getEffectorLocation().x;
				payloadPoint.y = stewartGough->getEffectorLocation().y;
				movementZ = true;
			}
			if(setValues.find("z") == -1) {
				// Probably a XY movement
				payloadPoint.z = stewartGough->getEffectorLocation().z;
			}
			*/
        /*}
        i++;
    }*/

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
		
		if(setValues.find("x")!= -1 && setValues.find("y")!= -1 
				&& setValues.find("z")!= -1){
			moveVector.set((translatedVector.x + rotatedX), (translatedVector.y + rotatedY), (translatedVector.z + payloadPoint.z));

		}
		/*
		if(movementZ) {
			// Z Movement
			moveVector.set(payloadPoint.x, payloadPoint.y, (translatedVector.z + payloadPoint.z));
		} else {
			// XY Movement
			moveVector.set((translatedVector.x + rotatedX), (translatedVector.y + rotatedY), payloadPoint.z);
			std::cout << "[MoveVectorX]: \t" << moveVector.x << "\n[MoveVectorY]: \t" << moveVector.y << std::endl;
		}
		*/
	} else {
		moveVector.set(payloadPoint.x, payloadPoint.y, payloadPoint.z);
	}

	//std::cout << "trying to move to x: " << moveVector.x << " y: " << moveVector.y << " z: " <<  moveVector.z << " with acceleration: " << payloadPoint.maxAcceleration << std::endl;
	if(moveToPoint(moveVector.x, moveVector.y, moveVector.z, rotationX, rotationY, rotationZ, payloadPoint.maxAcceleration)){
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
bool stewartGoughNodeNamespace::StewartGoughNode::calibrate(){
	if(!stewartGough->calibrateMotors()){
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
bool stewartGoughNodeNamespace::StewartGoughNode::moveToPoint(double x, double y, double z, double maxAcceleration){
	return moveToPoint(x, y, z, 0, 0, 0, maxAcceleration);
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
bool stewartGoughNodeNamespace::StewartGoughNode::moveToPoint(double x, double y, double z, double rotationX, double rotationY, double rotationZ, double maxAcceleration){

	if(maxAcceleration > 20){
		maxAcceleration = 20;
	}
	
	std::cout << "moveTo rotation: " << rotationX << " " << rotationY << " " << rotationZ << std::endl;
		
	
	rexos_datatypes::Point3D<double> oldLocation(stewartGough->getEffectorLocation());
	rexos_datatypes::Point3D<double> newLocation(x,y,z);

	try {
		stewartGough->moveTo(newLocation, maxAcceleration, rotationX, rotationY, rotationZ);
		return true;
	} catch(std::out_of_range& ex){
		return false;
	}
	
	
}







/**
 * Function that moves the stewart int that is relative to the current. 
 *
 * @param x destination relative x-coordinate
 * @param y destination relative y-coordinate
 * @param z destination relative z-coordinate
 * @param maxAcceleration maximum acceleration
 * 
 * @return false if the path is illegal, true if the motion is executed succesfully.
 **/
 
 /*
bool stewartGoughNodeNamespace::StewartGoughNode::moveToRelativePoint(double x, double y, double z, double maxAcceleration){
	rexos_datatypes::Point3D<double> oldLocation(stewartGough->getEffectorLocation());
	rexos_datatypes::Point3D<double> newLocation(x,y,z);
	newLocation += oldLocation;
	try {
		ROS_INFO("Moving to: (%f, %f, %f) maxAcceleration=%f", x, y, z, maxAcceleration);
		stewartGough->moveTo(newLocation, maxAcceleration);
		return true;
	} catch(std::out_of_range& ex){
		return false;
	}
	
}
*/



bool stewartGoughNodeNamespace::StewartGoughNode::transitionInitialize(){
	ROS_INFO("Initialize transition called");
	return true;
}

bool stewartGoughNodeNamespace::StewartGoughNode::transitionDeinitialize(){
	ROS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::transitionSetup(){
	ROS_INFO("Setup transition called");

	// Power on the deltarobot and calibrate the motors.
	stewartGough->powerOn();
	// Calibrate the motors

	if(!stewartGough->calibrateMotors()){
		ROS_ERROR("Calibration FAILED. EXITING.");
		return false;
	} else {
		return true;
	}
}

/**
 * Transition from Standby to Safe state
 * Will turn power off the motor 
 * @return will be 0 if everything went ok else error
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::transitionShutdown(){
	ROS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	stewartGough->powerOff();
	return true;
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::transitionStart(){
	ROS_INFO("Start transition called");
	//The service servers should be set, to provide the normal methods for the equiplet
	return true;
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::transitionStop(){
	ROS_INFO("Stop transition called");
	//The service servers should be set off, so the equiplet isn't able to set tasks for the module
		return true;
	// Go to base (Motors on 0 degrees)
}

/**
 * Parse a JSON string to a Point object
 *
 * @param json String that contains the json thats need to be parsed to a point
 *
 * @return Point object that is initialized from the data in the JSON
 **/
/*stewartGoughNodeNamespace::Point stewartGoughNodeNamespace::StewartGoughNode::parsePoint(const JSONNode & n, std::string * valuesSet){

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

		i++;
	}
	
	return p;
}

stewartGoughNodeNamespace::Point stewartGoughNodeNamespace::StewartGoughNode::parseLookup(const JSONNode & n){

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
		i++;
	}

	return p;
}

std::string stewartGoughNodeNamespace::StewartGoughNode::parseNodeValue(const std::string nodeName, const JSONNode & n){

	JSONNode::const_iterator i = n.begin();
	std::string result;
	while(i != n.end()){
		// get the JSON node name and value as a string
		std::string node_name = i->name();
		if(node_name == nodeName)
		{
			result = i->as_string();
		} 
		i++;
	}
	return result;
}*/


/**
 * Parse a JSON string to a Point object
 *
 * @param json String that contains the json thats need to be parsed to a point
 * @param size Reference to an int size, caller of the function needs to know this
 *
 * @return Point object that is initialized from the data in the JSON
 **/
stewartGoughNodeNamespace::Point* stewartGoughNodeNamespace::StewartGoughNode::parsePointArray(std::string json, int &size){
/*	ROS_INFO("Parsing JSON Array");
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
	return path;*/
}

/**
 * Main that creates the deltaRobotNode and starts the statemachine
 **/
int main(int argc, char **argv){
	ros::init(argc, argv, NODE_NAME);
	
	if(argc < 5){
		ROS_ERROR("Usage: stewart_gough_node equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	std::string equipletName = argv[1];
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(argv[2], argv[3], argv[4]);
	
	ROS_INFO("Creating StewartGoughNode");
	stewartGoughNodeNamespace::StewartGoughNode drn(equipletName, moduleIdentifier);

	ros::spin();
	return 0;
}
