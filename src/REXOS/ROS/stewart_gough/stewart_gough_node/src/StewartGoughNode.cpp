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
#include <execinfo.h>
#include <signal.h>

#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

#include <matrices/Matrices.h>

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
	REXOS_INFO("StewartGoughNode Constructor entering...");
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
	REXOS_INFO_STREAM("parsing hardwareStep: " << goal->json);
	Json::Reader reader;
	Json::Value equipletStepNode;
	reader.parse(goal->json, equipletStepNode);
	rexos_datatypes::EquipletStep equipletStep(equipletStepNode);
	
	rexos_statemachine::SetInstructionResult result;
	result.OID = goal->OID;
	
	rexos_stewart_gough::StewartGoughLocation origin;
	// the rotation of the axis of the tangent space against the normal space in radians
	double rotationX, rotationY, rotationZ = 0;
	
	// determine the position of the origin and the rotation of the axis
	switch(equipletStep.getOriginPlacement().getOriginPlacementType()) {
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_IDENTIFIER: {
			// set the origin to the result of the lookup
			if(equipletStep.getOriginPlacement().getLookupResult().isMember("location") == false) {
				throw std::runtime_error("lookup result does not contain location");
			} else if(equipletStep.getOriginPlacement().getLookupResult().isMember("rotation") == false) {
				throw std::runtime_error("lookup result does not contain rotation");
			}
			Json::Value location = equipletStep.getOriginPlacement().getLookupResult()["location"];
			origin.location.set(location["x"].asDouble(), location["y"].asDouble(), location["z"].asDouble());
			origin.location = convertToModuleCoordinate(origin.location);
			Json::Value rotation = equipletStep.getOriginPlacement().getLookupResult()["rotation"];
			rotationZ = rotation["z"].asDouble();
			break;
		}
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_CURRENT_POSITION: {
			// set the origin to the current position of the effector
			origin.location.set(stewartGough->getEffectorLocation().location.x, stewartGough->getEffectorLocation().location.y, 
					stewartGough->getEffectorLocation().location.z);
			origin.rotationX = stewartGough->getEffectorLocation().rotationX;
			origin.rotationY = stewartGough->getEffectorLocation().rotationY;
			origin.rotationZ = stewartGough->getEffectorLocation().rotationZ;
			break;
		}
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_MODULE_ORIGIN: {
			// set the origin to the origin of the module (eg set it to 0, 0, 0)
			origin.location.set(0, 0, 0);
			origin.rotationX = 0;
			origin.rotationY = 0;
			origin.rotationZ = 0;
			break;
		}
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_EQUIPLET_ORIGIN: {
			// set the origin to the origin of the module (eg set it to 0, 0, 0)
			origin.location = convertToModuleCoordinate(Vector3(0, 0, 0));
			origin.rotationX = 0;
			origin.rotationY = 0;
			origin.rotationZ = 0;
			break;
		}
	}
	
	// get the vector from the instruction data
	Json::Value instructionData = equipletStep.getInstructionData();
	if(instructionData.isMember("move") == false) {
		throw std::runtime_error("instruction data does not contain move");
	}
	Json::Value moveCommand = equipletStep.getInstructionData()["move"];
	Vector3 offsetVector;
	if(moveCommand.isMember("x")) offsetVector.x = moveCommand["x"].asDouble();
	else offsetVector.x = stewartGough->getEffectorLocation().location.x;
	if(moveCommand.isMember("y")) offsetVector.y = moveCommand["y"].asDouble();
	else offsetVector.y = stewartGough->getEffectorLocation().location.y;
	if(moveCommand.isMember("z")) offsetVector.z = moveCommand["z"].asDouble();
	else offsetVector.z = stewartGough->getEffectorLocation().location.z;
	
	// get the max acceleration
	double maxAcceleration;
	if(moveCommand.isMember("maxAcceleration") == false) {
		REXOS_WARN("move command does not contain maxAcceleration, assuming ");
		maxAcceleration = 50.0;
	} else {
		maxAcceleration = moveCommand["maxAcceleration"].asDouble();
	}
	
	// get the rotation from the instruction data
	if(instructionData.isMember("rotate") == false) {
		throw std::runtime_error("instruction data does not contain rotate");
	}
	Json::Value rotateCommand = equipletStep.getInstructionData()["rotate"];
	double targetRotationX, targetRotationY, targetRotationZ;
	if(rotateCommand.isMember("x")) targetRotationX = rotateCommand["x"].asDouble();
	else targetRotationX = stewartGough->getEffectorLocation().rotationX;
	if(rotateCommand.isMember("y")) targetRotationY = rotateCommand["y"].asDouble();
	else targetRotationY = stewartGough->getEffectorLocation().rotationY;
	if(rotateCommand.isMember("z")) targetRotationZ = rotateCommand["z"].asDouble();
	else targetRotationZ = stewartGough->getEffectorLocation().rotationZ;
	
	// calculate the target vector
	Matrix4 rotationMatrix;
	rotationMatrix.rotateX(rotationX);
	rotationMatrix.rotateY(rotationY);
	rotationMatrix.rotateZ(rotationZ);
	
	Vector3 targetVector = origin.location + rotationMatrix * offsetVector;
	targetRotationX += origin.rotationX;
	targetRotationY += origin.rotationY;
	targetRotationZ += origin.rotationZ;
	rexos_stewart_gough::StewartGoughLocation targetLocation(targetVector.x, targetVector.y, targetVector.z, 
			targetRotationX, targetRotationY, targetRotationZ);
	
	REXOS_INFO_STREAM("moving from " << stewartGough->getEffectorLocation() << " to " << targetLocation);
	
  	// finally move to point
	if(moveToPoint(targetLocation, maxAcceleration)) {
		setInstructionActionServer.setSucceeded(result);
	} else {
		REXOS_WARN("Failed moving to point");
		setInstructionActionServer.setAborted(result);
	}
}


// Calibrate service functions ------------------------------------------------
/**
 * Main function for starting the (re)calibratiion of the robot. Is called from the service functions. 
 *
 * @return true if the calibration was successful else false 
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::calibrate(){
	if(!stewartGough->calibrateMotors()){
		REXOS_ERROR("Calibration FAILED. EXITING.");
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
bool stewartGoughNodeNamespace::StewartGoughNode::moveToPoint(rexos_stewart_gough::StewartGoughLocation to, double maxAcceleration){
	if(maxAcceleration > 20){
		maxAcceleration = 20;
	}
	
	REXOS_INFO_STREAM("moveTo rotation: " << to.rotationX << " " << to.rotationY << " " << to.rotationZ << std::endl);
		
	
	rexos_stewart_gough::StewartGoughLocation oldLocation(stewartGough->getEffectorLocation());
	rexos_stewart_gough::StewartGoughLocation newLocation(to);

	try {
		stewartGough->moveTo(newLocation, maxAcceleration);
		return true;
	} catch(std::out_of_range& ex){
		REXOS_INFO_STREAM(ex.what());
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
		REXOS_INFO("Moving to: (%f, %f, %f) maxAcceleration=%f", x, y, z, maxAcceleration);
		stewartGough->moveTo(newLocation, maxAcceleration);
		return true;
	} catch(std::out_of_range& ex){
		return false;
	}
	
}
*/



bool stewartGoughNodeNamespace::StewartGoughNode::transitionInitialize(){
	REXOS_INFO("Initialize transition called");
	return true;
}

bool stewartGoughNodeNamespace::StewartGoughNode::transitionDeinitialize(){
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::transitionSetup(){
	REXOS_INFO("Setup transition called");

	// Power on the deltarobot and calibrate the motors.
	stewartGough->powerOn();
	// Calibrate the motors

	if(!stewartGough->calibrateMotors()){
		REXOS_ERROR("Calibration FAILED. EXITING.");
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
	REXOS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	stewartGough->powerOff();
	return true;
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::transitionStart(){
	REXOS_INFO("Start transition called");
	//The service servers should be set, to provide the normal methods for the equiplet
	return true;
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
bool stewartGoughNodeNamespace::StewartGoughNode::transitionStop(){
	REXOS_INFO("Stop transition called");
	//The service servers should be set off, so the equiplet isn't able to set tasks for the module
		return true;
	// Go to base (Motors on 0 degrees)
}


/**
 * Main that creates the deltaRobotNode and starts the statemachine
 **/
int main(int argc, char **argv){
	rexos_stewart_gough::SixAxisCalculations sc(100.00, 300.00, 
				50, 50, 
				20, 20,
				0.46);
	
	{
		rexos_stewart_gough::StewartGoughLocation location(Vector3(0, 0, -360), rexos_utilities::degreesToRadians(45), 0, 0.0);
		rexos_stewart_gough::SixAxisCalculations::EffectorMove movement = sc.getMotorAngles(location);
		REXOS_INFO_STREAM("----------");
		REXOS_INFO_STREAM(movement.validMove);
		REXOS_INFO_STREAM(movement.angles[0] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[1] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[2] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[3] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[4] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[5] / (2 * 3.14159263) * 360);
	}
	{
		rexos_stewart_gough::StewartGoughLocation location(Vector3(0, 0, -360), rexos_utilities::degreesToRadians(45), 0, 0.0);
		rexos_stewart_gough::SixAxisCalculations::EffectorMove movement = sc.getMotorAngles(location);
		REXOS_INFO_STREAM("----------");
		REXOS_INFO_STREAM(movement.validMove);
		REXOS_INFO_STREAM(movement.angles[0] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[1] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[2] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[3] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[4] / (2 * 3.14159263) * 360);
		REXOS_INFO_STREAM(movement.angles[5] / (2 * 3.14159263) * 360);
	}
	
	ros::init(argc, argv, NODE_NAME);
	
	if(argc < 5){
		REXOS_ERROR("Usage: stewart_gough_node equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	std::string equipletName = argv[1];
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(argv[2], argv[3], argv[4]);
	
	REXOS_INFO("Creating StewartGoughNode");
	stewartGoughNodeNamespace::StewartGoughNode drn(equipletName, moduleIdentifier);

	ros::spin();
	return 0;
}
