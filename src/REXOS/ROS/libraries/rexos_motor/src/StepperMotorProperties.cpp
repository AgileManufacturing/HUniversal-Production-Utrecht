/**
 * @file StepperMotorProperties.cpp
 * @brief Contains the properties of a stepper motor.
 * @date Created: 2012-10-02
 *
 * @author Tommas Bakker
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

#include <rexos_motor/StepperMotorProperties.h>

#include <cmath>
#include <iostream>

#include <rexos_utilities/Utilities.h>

#include "ros/ros.h"

namespace rexos_motor {
	/**
	 * Constructor of StepperMotor. Sets angles to unlimited.
	 *
	 * @param modbusController Controller for the modbus communication.
	 * @param motorIndex Index of the motor from 0 to N dependant on the amount of motors.
	 * @param minAngle Minimum for the angle, in radians, the StepperMotor can travel on the theoretical plane.
	 * @param maxAngle Maximum for the angle, in radians, the StepperMotor can travel on the theoretical plane.
	 **/
	StepperMotorProperties::StepperMotorProperties(JSONNode properties) {
		for(JSONNode::const_iterator it = properties.begin(); it != properties.end(); it++) {
			if(it->name() == "motorMinAngleDegrees"){
				motorMinAngle = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found motorMinAngle " << motorMinAngle);
			} else if(it->name() == "motorMaxAngleDegrees"){
				motorMaxAngle = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found motorMaxAngle " << motorMaxAngle);
			
			} else if(it->name() == "microStepAngleDegrees"){
				microStepAngle = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found microStepAngle " << microStepAngle);
			
			} else if(it->name() == "minAccelerationDegrees"){
				minAcceleration = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found minAcceleration " << minAcceleration);
			} else if(it->name() == "maxAccelerationDegrees"){
				maxAcceleration = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found maxAcceleration " << maxAcceleration);
			
			} else if(it->name() == "minSpeedDegrees"){
				minSpeed = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found minSpeed " << minSpeed);
			} else if(it->name() == "maxSpeedDegrees"){
				maxSpeed = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found maxSpeed " << maxSpeed);
			} else {
				// some other property, ignore it
			}
		}
	}
/*	JSONNode StepperMotorProperties::findJsonNode(JSONNode input, std::string name) {
		for(JSONNode::const_iterator it = input.begin(); it != input.end(); it++) {
			if(it->name() == name) {
				return it->as_node();
			}
		}
		throw std::runtime_error("JSONNode not found by name " + name + " in " input.write_formatted());
	}*/
}
