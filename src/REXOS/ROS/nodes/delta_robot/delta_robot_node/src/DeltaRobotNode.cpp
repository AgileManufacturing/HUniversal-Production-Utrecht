/**
 * @file DeltaRobotNode.cpp
 * @brief Provide the services to move the DeltaRobot.
 * @date Created: 2012-09-19
 *
 * @author Dick van der Steen
 * @author Dennis Koole
 * @author Alexander Streng
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

#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>

#include <matrices/Matrices.h>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "DeltaRobotNode"
// @endcond

/**
 * Constructor
 * @param equipletID identifier for the equiplet
 * @param moduleID identifier for the deltarobot
 **/
deltaRobotNodeNamespace::DeltaRobotNode::DeltaRobotNode(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier) :
	rexos_knowledge_database::Module(moduleIdentifier),
	rexos_statemachine::ModuleStateMachine(equipletName, moduleIdentifier, true),
	rexos_coordinates::Module(this),
	deltaRobot(NULL),
	setInstructionActionServer(
	    nodeHandle,
	    equipletName + "/" + moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber() + "/set_instruction",
	    boost::bind(&deltaRobotNodeNamespace::DeltaRobotNode::onSetInstruction, this, _1),
	    false),
	lastX(0.0),
	lastY(0.0),
	lastZ(0.0)
{
	ROS_INFO("DeltaRobotnode Constructor entering...");
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

	// Create a deltarobot
	deltaRobot = new rexos_delta_robot::DeltaRobot(jsonNode);

	setInstructionActionServer.start();

	ROS_INFO_STREAM("DeltaRobotNode initialized. Advertising actionserver on " <<
	                moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber() <<
	                "/set_instruction");
}



deltaRobotNodeNamespace::DeltaRobotNode::~DeltaRobotNode()
{
	delete deltaRobot;
}


void deltaRobotNodeNamespace::DeltaRobotNode::onSetInstruction(const rexos_statemachine::SetInstructionGoalConstPtr &goal)
{
	ROS_INFO_STREAM("parsing hardwareStep: " << goal->json);
	Json::Reader reader;
	Json::Value equipletStepNode;
	reader.parse(goal->json, equipletStepNode);
	rexos_datatypes::EquipletStep equipletStep(equipletStepNode);
	
	rexos_statemachine::SetInstructionResult result;
	result.OID = goal->OID;

	Vector4 origin;
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
			origin.set(location["x"].asDouble(), location["y"].asDouble(), location["z"].asDouble(), 1);
			origin = convertToModuleCoordinate(origin);
			Json::Value rotation = equipletStep.getOriginPlacement().getLookupResult()["rotation"];
			rotationZ = rotation["z"].asDouble();
			break;
		}
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_CURRENT_POSITION: {
			// set the origin to the current position of the effector
			// TODO get rid of Point3D
			origin.set(deltaRobot->getEffectorLocation().x, deltaRobot->getEffectorLocation().y, deltaRobot->getEffectorLocation().z, 1);
			break;
		}
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_MODULE_ORIGIN: {
			// set the origin to the origin of the module (eg set it to 0, 0, 0)
			origin.set(0, 0, 0, 1);
			break;
		}
		case rexos_datatypes::OriginPlacement::RELATIVE_TO_EQUIPLET_ORIGIN: {
			// set the origin to the origin of the module (eg set it to 0, 0, 0)
			origin = convertToModuleCoordinate(Vector4(0, 0, 0, 1));
			break;
		}
	}
	
	// get the vector from the instruction data
	Json::Value instructionData = equipletStep.getInstructionData();
	if(instructionData.isMember("move") == false) {
		throw std::runtime_error("instruction data does not contain move");
	}
	Json::Value moveCommand = equipletStep.getInstructionData()["move"];
	Vector4 offsetVector;
	if(moveCommand.isMember("x")) offsetVector.x = moveCommand["x"].asDouble();
	else offsetVector.x = deltaRobot->getEffectorLocation().x;
	if(moveCommand.isMember("y")) offsetVector.y = moveCommand["y"].asDouble();
	else offsetVector.y = deltaRobot->getEffectorLocation().y;
	if(moveCommand.isMember("z")) offsetVector.z = moveCommand["z"].asDouble();
	else offsetVector.z = deltaRobot->getEffectorLocation().z;
	
	// get the max acceleration
	double maxAcceleration;
	if(moveCommand.isMember("maxAcceleration") == false) {
		ROS_WARN("move command does not contain maxAcceleration, assuming ");
		maxAcceleration = 50.0;
	} else {
		maxAcceleration = moveCommand["maxAcceleration"].asDouble();
	}
	
	// calculate the target vector
	Matrix4 rotationMatrix;
	rotationMatrix.rotateX(rotationX);
	rotationMatrix.rotateY(rotationY);
	rotationMatrix.rotateZ(rotationZ);
	
	Vector4 targetVector = origin + rotationMatrix * offsetVector;
	
	ROS_INFO_STREAM("moving from " << deltaRobot->getEffectorLocation() << " to " << targetVector);

	if(moveToPoint(targetVector.x, targetVector.y, targetVector.z, maxAcceleration))
	{
		setInstructionActionServer.setSucceeded(result);
	} else {
		ROS_WARN("Failed moving to point");
		setInstructionActionServer.setAborted(result);
	}
}


// Calibrate service functions ------------------------------------------------
/**
 * Main function for starting the (re)calibratiion of the robot. Is called from the service functions.
 *
 * @return true if the calibration was successful else false
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::calibrate()
{
	if(!deltaRobot->calibrateMotors()) {
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
bool deltaRobotNodeNamespace::DeltaRobotNode::moveToPoint(double x, double y, double z, double maxAcceleration)
{
	Vector3 oldLocation(deltaRobot->getEffectorLocation());
	Vector3 newLocation(x,y,z);

	if(deltaRobot->checkPath(oldLocation, newLocation)) {
		deltaRobot->moveTo(newLocation, maxAcceleration);
		return true;
	}
	return false;
}


bool deltaRobotNodeNamespace::DeltaRobotNode::transitionInitialize()
{
	ROS_INFO("Initialize transition called");

	return true;
}

bool deltaRobotNodeNamespace::DeltaRobotNode::transitionDeinitialize()
{
	ROS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}

/**
 * Transition from Safe to Standby state
 * @return 0 if everything went OK else error
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::transitionSetup()
{
	ROS_INFO("Setup transition called");
	// Generate the effector boundaries with voxel size 2
	deltaRobot->generateBoundaries(2);
	// Power on the deltarobot and calibrate the motors.
	deltaRobot->powerOn();
	// Calibrate the motors
	if(!deltaRobot->calibrateMotors()) {
		ROS_ERROR("Calibration FAILED. EXITING.");
		return false;
	} else {
		rexos_statemachine::TransitionGoal goal;
		goal.gainedSupportedMutations.push_back("move");

		transitionActionClient.sendGoal(goal);
		return true;
	}
}

/**
 * Transition from Standby to Safe state
 * Will turn power off the motor
 * @return will be 0 if everything went ok else error
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::transitionShutdown()
{
	ROS_INFO("Shutdown transition called");
	// Should have information about the workspace, calculate a safe spot and move towards it
	deltaRobot->powerOff();
	return true;
}

/**
 * Transition from Standby to Normal state
 * @return will be 0 if everything went ok else error
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::transitionStart()
{
	ROS_INFO("Start transition called");
	//The service servers should be set, to provide the normal methods for the equiplet
	return true;
}
/**
 * Transition from Normal to Standby state
 * @return will be 0 if everything went ok else error
 **/
bool deltaRobotNodeNamespace::DeltaRobotNode::transitionStop()
{
	ROS_INFO("Stop transition called");
	//The service servers should be set off, so the equiplet isn't able to set tasks for the module
	return true;
	// Go to base (Motors on 0 degrees)
}


/**
 * Main that creates the deltaRobotNode and starts the statemachine
 **/
int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	if(argc < 5) {
		ROS_ERROR("Usage: delta_robot_node equipletName manufacturer typeNumber serialNumber");
		return -1;
	}

	std::string equipletName = argv[1];
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(argv[2], argv[3], argv[4]);

	ROS_INFO("Creating DeltaRobotNode");
	deltaRobotNodeNamespace::DeltaRobotNode drn(equipletName, moduleIdentifier);

	ros::spin();
	return 0;
}
