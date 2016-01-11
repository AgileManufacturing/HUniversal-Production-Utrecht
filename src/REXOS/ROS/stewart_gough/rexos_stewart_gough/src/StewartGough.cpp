/**
 * @file DeltaRobot.cpp
 * @brief Symbolizes an entire deltarobot.
 *
 * @author Garik Hakopian
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


#include <rexos_stewart_gough/StewartGough.h>
#include <rexos_utilities/Utilities.h>
#include "rexos_stewart_gough/SixAxisCalculations.h"
#include "rexos_logger/rexos_logger.h"

#include <string>
#include <stdexcept>
#include <vector>

namespace rexos_stewart_gough{
	/**
	 * Constructor of a sixaxis robot.
	 * 
	 * @param steqartGoughMeasures The measures of the steqart gough configuration in use.
	 * @param motorManager The manager that allows all motors to be simultaneously activated.
	 * @param motors The motor array with the three motor objects.
	 * @param modbusIO The TCP modbus connection for the IO controller.
	 **/
	StewartGough::StewartGough(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, Json::Value node) :
			MotorizedActor(equipletName, identifier, isSimulated, 6, node),
			stewartGoughMeasures(node["stewartGoughMeasures"]), 
			effectorLocation() {
		REXOS_INFO("StewartGough constructor entering...");
		readJSONNode(node);
		sixAxisCalculations = new SixAxisCalculations(stewartGoughMeasures);

		REXOS_INFO("end of constructor reached");
	}

	StewartGough::~StewartGough(void){
	}
	
	void StewartGough::readJSONNode(const Json::Value node) {
	}
	

	/**
	 * Checks the path between two points.
	 * 
	 * @param begin The starting point.
	 * @param end The end point.
	 * 
	 * @return if the path between two points is valid.
	 **/
	bool StewartGough::checkPath(const StewartGoughLocation& begin, const StewartGoughLocation& end) {
		return sixAxisCalculations->checkPath(begin, end);
	}
			
	
	

	/**
	 * Gets the acceleration in radians/s² for a motor rotation with a certain relative angle and time, which is half acceleration and half deceleration (there is no period of constant speed).
	 * 
	 * @param moveTime the move time.
	 * @param relativeAngle The relative angle
	 *
	 * @return the acceleration in radians/s²
	 **/
	double StewartGough::getAccelerationForRotation(double relativeAngle, double moveTime){
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
	double StewartGough::getSpeedForRotation(double relativeAngle, double moveTime, double acceleration){
		return (acceleration/2) * (moveTime - sqrt((moveTime * moveTime) - (4 * fabs(relativeAngle) / acceleration)));
	}


	void StewartGough::moveTo(StewartGoughLocation point, double maxAcceleration){
		// check whether the motors are powered on.
		if(!motorManager->isPoweredOn()){
			throw std::runtime_error("motor drivers are not powered on");
		}

		//check if the requested location is the effectors current location,
		//if so the method can be cut short.
		if(effectorLocation == point) {
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
		SixAxisCalculations::EffectorMove effectorMove = sixAxisCalculations->getMotorAngles(point);
		if(effectorMove.validMove == false) {
			throw std::out_of_range("move is not valid because joints are out of bounds or position is out of reach");
		}
		std::vector<rexos_motor::MotorRotation> rotations;
		for(uint i = 0; i < motors.size(); i++){
			rexos_motor::MotorRotation rotation;
			rotation.angle = effectorMove.angles[i];
			rotations.push_back(rotation);
		}

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

	/**
	* Calibrates all six motors of the deltarobot by moving the motors upwards one by one.
	* After a motor is moved upwards, it is moved back to the 0 degrees state.
	* This function temporarily removes the limitations for the motorcontrollers.
	* 
	* @return true if the calibration was succesful. False otherwise (e.g. failure on sensors.)
	**/
	bool StewartGough::calibrateMotors(){
		/*for(uint i = 0; i < motors.size(); i += 2) {
			std::vector<rexos_motor::MotorInterface*> motorsToCalibrate;
			std::vector<rexos_sensor::ContactSensor*> sensorsToUse;
			motorsToCalibrate.push_back(motors[i + 0]);
			motorsToCalibrate.push_back(motors[i + 1]);
			sensorsToUse.push_back(sensors[i + 0]);
			sensorsToUse.push_back(sensors[i + 1]);
			calibrateMotorGroup(motorsToCalibrate, sensorsToUse);
		}*/
		calibrateMotorGroup(motors, sensors);
		
		effectorLocation.location.x = 0;
		effectorLocation.location.y = 0;
		effectorLocation.location.z = -sqrt(
					pow(stewartGoughMeasures.ankleLength, 2) - 
					pow(stewartGoughMeasures.baseRadius + stewartGoughMeasures.hipLength - stewartGoughMeasures.effectorRadius, 2)
				) - stewartGoughMeasures.effectorHeight;
		effectorLocation.rotationX = 0;
		effectorLocation.rotationY = 0;
		effectorLocation.rotationZ = 0;
		REXOS_DEBUG_STREAM("effector location z: " << effectorLocation.location.z); 
		
		return true;
	}

	/**
	 * Get the location of the midpoint of the effector.
	 *
	 * @return The coordinate for the midpoint of the effector.
	 **/
	StewartGoughLocation StewartGough::getEffectorLocation(){
		return effectorLocation;
	}
	
}
