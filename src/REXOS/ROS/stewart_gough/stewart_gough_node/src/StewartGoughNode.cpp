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

using namespace stewart_gough_node;
/**
 * Constructor 
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the deltarobot
 **/
StewartGoughNode::StewartGoughNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, bool isShadow) :
		rexos_module::ActorModule::ActorModule(equipletName, moduleIdentifier, isSimulated, isShadow),
		lastX(0.0),
		lastY(0.0),
		lastZ(0.0),
		stewartGough(NULL) {
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
	for(uint i = 0; i < typeJsonNodeMemberNames.size(); i++) {
		jsonNode[typeJsonNodeMemberNames[i]] = typeJsonNode[typeJsonNodeMemberNames[i]];
	}

	Json::StyledWriter writer;
	ROS_INFO("%s", writer.write(jsonNode).c_str());
		
	// Create a stewart gough robot
	REXOS_INFO_STREAM("Creating new stewart");
	stewartGough = new rexos_stewart_gough::StewartGough(equipletName, moduleIdentifier, isSimulated, jsonNode);
}


StewartGoughNode::~StewartGoughNode() {
	delete stewartGough;
}


void StewartGoughNode::onExecuteHardwareStep(const rexos_module::ExecuteHardwareStepGoalConstPtr &goal) {
	REXOS_INFO_STREAM("at hardwarestep now");
	REXOS_INFO_STREAM("parsing hardwareStep: " << goal->json);
	Json::Reader reader;
	Json::Value equipletStepNode;
	reader.parse(goal->json, equipletStepNode);
	rexos_datatypes::HardwareStep equipletStep(equipletStepNode);
	
	rexos_module::ExecuteHardwareStepResult result;
	result.OID = goal->OID;
	
	rexos_stewart_gough::StewartGoughLocation origin;
	// the rotation of the axis of the tangent space against the normal space in radians
	double rotationX = 0, rotationY = 0, rotationZ = 0;
	
	// determine the position of the origin and the rotation of the axis
	switch(equipletStep.getOriginPlacement().getOriginPlacementType()) {
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_PART_ORIGIN: {
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
		default: {
			throw std::invalid_argument("equipletStep::originPlacement was of unknown type");
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
		REXOS_WARN("move command does not contain maxAcceleration, assuming 50.0");
		maxAcceleration = 50.0;
	} else {
		maxAcceleration = moveCommand["maxAcceleration"].asDouble();
	}
	
	double targetRotationX, targetRotationY, targetRotationZ;
	// get the rotation from the instruction data
	if(instructionData.isMember("rotate") == false) {
		REXOS_WARN("instruction data does not contain rotate, assuming 0 0");
		targetRotationX = 0;
		targetRotationY = 0;
		targetRotationZ = 0;
	} else {
		Json::Value rotateCommand = equipletStep.getInstructionData()["rotate"];
		if(rotateCommand.isMember("x")) targetRotationX = rotateCommand["x"].asDouble();
		else targetRotationX = stewartGough->getEffectorLocation().rotationX;
		if(rotateCommand.isMember("y")) targetRotationY = rotateCommand["y"].asDouble();
		else targetRotationY = stewartGough->getEffectorLocation().rotationY;
		if(rotateCommand.isMember("z")) targetRotationZ = rotateCommand["z"].asDouble();
		else targetRotationZ = stewartGough->getEffectorLocation().rotationZ;
	}
	
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
		executeHardwareStepServer.setSucceeded(result);
	} else {
		REXOS_WARN("Failed moving to point");
		executeHardwareStepServer.setAborted(result);
	}
}


// Calibrate service functions ------------------------------------------------
/**
 * Main function for starting the (re)calibratiion of the robot. Is called from the service functions. 
 *
 * @return true if the calibration was successful else false 
 **/
bool StewartGoughNode::calibrate(){
	REXOS_INFO_STREAM("Calibrate called");
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
bool StewartGoughNode::moveToPoint(rexos_stewart_gough::StewartGoughLocation to, double maxAcceleration){
	REXOS_INFO_STREAM("MoveToPoint called");
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

bool StewartGoughNode::transitionInitialize(){
	REXOS_INFO("Initialize transition called");
	return true;
}

bool StewartGoughNode::transitionDeinitialize(){
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
bool StewartGoughNode::transitionSetup(){
	REXOS_INFO("Setup transition called");

	// Power on the deltarobot and calibrate the motors.
	stewartGough->powerOn();
	// Calibrate the motors

	if(!stewartGough->calibrateMotors()){
		REXOS_ERROR("Calibration FAILED. EXITING.");
		return false;
	}
	rexos_module::TransitionGoal goal;
	goal.gainedSupportedMutations.push_back("move");
	
	transitionActionClient.sendGoal(goal);
	return true;
}

/**
 * Transition from Standby to Safe state
 * Will turn power off the motor 
 * @return will be 0 if everything went ok else error
 **/
bool StewartGoughNode::transitionShutdown(){
	REXOS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	stewartGough->powerOff();
	return true;
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error 
 **/
bool StewartGoughNode::transitionStart(){
	REXOS_INFO("Start transition called");
	//The service servers should be set, to provide the normal methods for the equiplet
	return true;
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
bool StewartGoughNode::transitionStop(){
	REXOS_INFO("Stop transition called");
	//The service servers should be set off, so the equiplet isn't able to set tasks for the module
		return true;
	// Go to base (Motors on 0 degrees)
}


/**
 * Main that creates the deltaRobotNode and starts the statemachine
 **/
int main(int argc, char **argv){
	/*	rexos_stewart_gough::SixAxisCalculations sc(100.00, 300.00, 
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
	}*/
	
	
	if(argc < 5){
		REXOS_ERROR("Usage: stewart_gough_node (--isSimulated | --isShadow) equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	bool isSimulated = false;
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--isSimulated") {
			isSimulated = true;
		} else if (arg == "--isShadow") {
			isShadow = true;
			isSimulated = true;
		}
	}
	
	std::string equipletName = std::string(argv[argc - 4]);
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[argc - 3], argv[argc - 2], argv[argc - 1]);
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_" + moduleIdentifier.getManufacturer() + "_" + 
			moduleIdentifier.getTypeNumber() + "_" + moduleIdentifier.getSerialNumber();
	ros::init(argc, argv, nodeName);
	
	REXOS_INFO("Creating StewartGoughNode");
	StewartGoughNode drn(equipletName, moduleIdentifier, isSimulated, isShadow);

	ros::spin();
	return 0;
}
