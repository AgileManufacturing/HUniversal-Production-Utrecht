/**
 * @file DeltaRobot.cpp
 * @brief Symbolizes an entire deltarobot.
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Koen Braham
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

#include <sstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <stdexcept>
#include <cmath>

#include <rexos_delta_robot/DeltaRobot.h>
#include <rexos_delta_robot/EffectorBoundaries.h>
#include <rexos_delta_robot/InverseKinematics.h>
#include <rexos_delta_robot/InverseKinematicsException.h>
#include <rexos_utilities/Utilities.h>

#include "ros/ros.h"

namespace rexos_delta_robot {
	/**
	 * Constructor of a deltarobot.
	 * 
	 * @param deltaRobotMeasures The measures of the deltarobot configuration in use.
	 * @param motorManager The manager that allows all motors to be simultaneously activated.
	 * @param motors The motor array with the three motor objects.
	 * @param modbusIO The TCP modbus connection for the IO controller.
	 **/
	DeltaRobot::DeltaRobot(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, Json::Value node) :
			MotorizedActor(equipletName, moduleIdentifier, isSimulated, 3, node),
			kinematics(NULL),
			boundaries(NULL),
			effectorLocation(Vector3(0, 0, 0)), 
			boundariesGenerated(false) {
		REXOS_INFO("DeltaRobot constructor entering...");
		readJSONNode(node);
		kinematics = new InverseKinematics(*deltaRobotMeasures);
	}

	/**
	 * Deconstructor of a deltarobot. Turns off the motors and deletes the kinematics model.
	 **/
	DeltaRobot::~DeltaRobot(void) {
		delete kinematics;
	}
	
	void DeltaRobot::readJSONNode(const Json::Value node) {
		Json::Value deltaRobotNode = node["deltaRobotMeasures"];
		deltaRobotMeasures = new rexos_delta_robot::DeltaRobotMeasures(deltaRobotNode);
		REXOS_INFO_STREAM("found deltraRobotMeasures");
	}
	/**
	 * Generates the effectorBoundaries for the given voxelSize.
	 *
	 * @param voxelSize The size in millimeters of a side of a voxel in the boundaries.
	 **/
	void DeltaRobot::generateBoundaries(double voxelSize) {
		boundaries = rexos_delta_robot::EffectorBoundaries::generateEffectorBoundaries((*kinematics), deltaRobotMeasures, motors, voxelSize);
		boundariesGenerated = true;
	}
	
	/**
	 * Checks the path between two points.
	 * 
	 * @param begin The starting point.
	 * @param end The end point.
	 * 
	 * @return if the path between two points is valid.
	 **/
	bool DeltaRobot::checkPath(const Vector3& begin, const Vector3& end) {
		return boundaries->checkPath(begin, end);
	}

	/**
	 * Gets the acceleration in radians/s² for a motor rotation with a certain relative angle and time, which is half acceleration and half deceleration (there is no period of constant speed).
	 * 
	 * @param relativeAngle The relative angle
	 * @param moveTime the move time.
	 *
	 * @return the acceleration in radians/s²
	 **/
	double DeltaRobot::getAccelerationForRotation(double relativeAngle, double moveTime) {
		return (4 * fabs(relativeAngle)) / (moveTime * moveTime);
	}

	/**
	 * Gets the top speed in radians/s for a motor rotation with a certain relative angle, time and acceleration.
	 * 
	 * @param relativeAngle The relative angle
	 * @param moveTime the move time.
	 * @param acceleration the acceleration
	 *
	 * @return the speed in radians/s
	 **/
	double DeltaRobot::getSpeedForRotation(double relativeAngle, double moveTime, double acceleration) {
		return (acceleration/2) * (moveTime - sqrt((moveTime * moveTime) - (4 * fabs(relativeAngle) / acceleration)));
	}

	/**
	 * Makes the deltarobot move to a point.
	 * 
	 * @param point 3-dimensional point to move to.
	 * @param maxAcceleration the acceleration in radians/s² that the motor with the biggest motion will accelerate at.
	 **/
	void DeltaRobot::moveTo(const Vector3& point, double maxAcceleration) {
		// check whether the motors are powered on.
		if(!motorManager->isPoweredOn()) {
			throw std::runtime_error("motor drivers are not powered on");
		}

		if(effectorLocation == point) {
			// The effector is already at the requested location, the method can be cut short.
			return;
		}

		if(maxAcceleration > stepperMotorProperties.maxAcceleration) {
			// The acceleration is too high, putting it down to the maximum CRD514KD acceleration.
			maxAcceleration = stepperMotorProperties.maxAcceleration;
		} else if(maxAcceleration < stepperMotorProperties.minAcceleration) {
			// The acceleration is too low, throwing an exception.
			throw std::out_of_range("maxAcceleration too low");
		}

		// Get the motor angles from the kinematics model
		std::vector<rexos_motor::MotorRotation> rotations = kinematics->destinationPointToMotorRotations(point);

		// Check if the angles fit within the boundaries
		for(uint i = 0; i < motors.size(); i++) {
			if(rotations[i].angle < stepperMotorProperties.motorMinAngle || rotations[i].angle > stepperMotorProperties.motorMaxAngle) {
				throw std::out_of_range("motion angles outside of valid range");
			}
		}

		// Check if the path fits within the boundaries
		if(!checkPath(effectorLocation, point)) {
			throw std::out_of_range("invalid path");
		}

		// An array to hold the relative angles for the motors
		std::vector<bool> relativeAngles;
		for(uint i = 0; i < motors.size(); i++) {
			relativeAngles.push_back(0.0);
		}
		
		// An array that indicates for each motor whether it moves in this motion or not.
		std::vector<bool> motorIsMoved;
		for(uint i = 0; i < motors.size(); i++) {
			motorIsMoved.push_back(true);
		}
		
		// Index for the motor with the biggest motion
		uint motorWithBiggestMotion = 0;

		for(uint i = 0; i < motors.size(); i++) {
			relativeAngles[i] = fabs(rotations[i].angle - motors[i]->getCurrentAngle());
			if (relativeAngles[i] > relativeAngles[motorWithBiggestMotion]){
				motorWithBiggestMotion = i;
			}

			if(relativeAngles[i] < stepperMotorProperties.microStepAngle){
				// motor does not have to move at all
				motorIsMoved[i] = false;
			}
		}
		
		// Check if any motor has to move at all
		bool anyMotorIsMoved = false;
		for(uint i = 0; i < motors.size(); i++) {
			if(motorIsMoved[i] == true) {
				anyMotorIsMoved = true;
			}
		}
		if(anyMotorIsMoved == false) {
			// none of the motors have to move, method can be cut short
			return;
		}

		// switch currentMotionSlot
		/*currentMotionSlot++;
		if(currentMotionSlot > rexos_motor::CRD514KD::MOTION_SLOTS_USED){
			currentMotionSlot = 1;
		}*/

		// Set the acceleration of the motor with the biggest motion to the given maximum.
		rotations[motorWithBiggestMotion].acceleration = maxAcceleration;
		rotations[motorWithBiggestMotion].deceleration = maxAcceleration;

		// Calculate the time the motion will take, based on the assumption that the motion is two-phase (half acceleration and half deceleration).
		// TODO: Take the motor's maximum speed into account.
		double moveTime;

		if(sqrt(relativeAngles[motorWithBiggestMotion] * rotations[motorWithBiggestMotion].acceleration) > stepperMotorProperties.maxSpeed) {
			// In case of a two-phase motion, the top speed would come out above the motor's maximum, so a three-phase motion must be made.
			rotations[motorWithBiggestMotion].speed = stepperMotorProperties.maxSpeed;
			moveTime = (relativeAngles[motorWithBiggestMotion] / rotations[motorWithBiggestMotion].speed) + 
					(rotations[motorWithBiggestMotion].speed / rotations[motorWithBiggestMotion].acceleration);  
		} else {
			// The motion is fine as a two-phase motion.
			moveTime = 2 * sqrt(relativeAngles[motorWithBiggestMotion] / rotations[motorWithBiggestMotion].acceleration);
		}
		
		// Set speed, and also the acceleration for the smaller motion motors
		for(uint i = 0; i < motors.size(); i++) {
			rotations[i].speed = stepperMotorProperties.maxSpeed;

			if(i != motorWithBiggestMotion) {
				if(motorIsMoved[i]) {
					rotations[i].acceleration = getAccelerationForRotation(relativeAngles[i], moveTime);
					rotations[i].deceleration = rotations[i].acceleration;  
					if(rotations[i].acceleration < stepperMotorProperties.minAcceleration) {
						// The acceleration comes out too low, this means the motion cannot be half acceleration and half deceleration (without a consant speed phase).
						// To make it comply with the move time, as well as the minimum acceleration requirements, we have to add a top speed.
						rotations[i].acceleration = stepperMotorProperties.minAcceleration;
						rotations[i].deceleration = stepperMotorProperties.minAcceleration;
						rotations[i].speed = getSpeedForRotation(relativeAngles[i], moveTime, rotations[i].acceleration);
					} else if(rotations[i].acceleration > stepperMotorProperties.maxAcceleration) {
						throw std::out_of_range("acceleration too high");
					}
				} else {
					rotations[i].acceleration = stepperMotorProperties.minAcceleration;
					rotations[i].deceleration = stepperMotorProperties.minAcceleration;
					rotations[i].angle = motors[i]->getCurrentAngle();
				}
			}
			motors[i]->writeRotationData(rotations[i]);
		}

		motorManager->startMovement();
		
		effectorLocation = point;
	}
	
	bool DeltaRobot::calibrateMotors() {
		calibrateMotorGroup(motors, sensors);

		effectorLocation.x = 0;
		effectorLocation.y = 0;
		effectorLocation.z = -sqrt((deltaRobotMeasures->ankle * deltaRobotMeasures->ankle) - ((
				deltaRobotMeasures->base + deltaRobotMeasures->hip - deltaRobotMeasures->effector) * (
				deltaRobotMeasures->base + deltaRobotMeasures->hip - deltaRobotMeasures->effector))
		);
		REXOS_DEBUG_STREAM("effector location z: " << effectorLocation.z); 

		return true;
	}

	/**
	 * Get the location of the midpoint of the effector.
	 *
	 * @return The coordinate for the midpoint of the effector.
	 **/
	Vector3 DeltaRobot::getEffectorLocation(){
		return effectorLocation;
	}
}
