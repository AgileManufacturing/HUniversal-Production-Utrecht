/**
 * @file DeltaRobotMeasures.cpp
 * @brief Holds all measures for the DeltaRobot
 * @date Created: 2012-10-02
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

#include <rexos_stewart_gough/StewartGoughMeasures.h>
#include <cmath>
#include <iostream>
#include <rexos_utilities/Utilities.h>
#include "ros/ros.h"

namespace rexos_stewart_gough {
	StewartGoughMeasures::StewartGoughMeasures(JSONNode properties) {
		for(JSONNode::const_iterator it = properties.begin(); it != properties.end(); it++) {
			if(it->name() == "baseRadius"){
				base = it->as_float();
				ROS_INFO_STREAM("found baseRadius " << base);
			} else if(it->name() == "hipLength"){
				hip = it->as_float();
				ROS_INFO_STREAM("found hipLength " << hip);
			} else if(it->name() == "effectorRadius"){
				effector = it->as_float();
				ROS_INFO_STREAM("found effectorRadius " << effector);
			} else if(it->name() == "ankleLength"){
				ankle = it->as_float();
				ROS_INFO_STREAM("found ankleLength " << ankle);
			
			} else if(it->name() == "hipAnleMaxAngleDegrees"){
				maxAngleHipAnkle = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found maxAngleHipAnkle " << maxAngleHipAnkle);
			} else if(it->name() == "motorFromZeroToTopAngleDegrees"){
				motorFromZeroToTopAngle = rexos_utilities::degreesToRadians(it->as_float());
				ROS_INFO_STREAM("found motorFromZeroToTopAngle " << motorFromZeroToTopAngle);
			
			} else if(it->name() == "boundaryBoxMinX"){
				boundaryBoxMinX = it->as_float();
				ROS_INFO_STREAM("found boundaryBoxMinX " << boundaryBoxMinX);
			} else if(it->name() == "boundaryBoxMaxX"){
				boundaryBoxMaxX = it->as_float();
				ROS_INFO_STREAM("found boundaryBoxMaxX " << boundaryBoxMaxX);
			} else if(it->name() == "boundaryBoxMinY"){
				boundaryBoxMinY = it->as_float();
				ROS_INFO_STREAM("found boundaryBoxMinY " << boundaryBoxMinY);
			} else if(it->name() == "boundaryBoxMaxY"){
				boundaryBoxMaxY = it->as_float();
				ROS_INFO_STREAM("found boundaryBoxMaxY " << boundaryBoxMaxY);
			} else if(it->name() == "boundaryBoxMinZ"){
				boundaryBoxMinZ = it->as_float();
				ROS_INFO_STREAM("found boundaryBoxMinZ " << boundaryBoxMinZ);
			} else if(it->name() == "boundaryBoxMaxZ"){
				boundaryBoxMaxZ = it->as_float();
				ROS_INFO_STREAM("found boundaryBoxMaxZ " << boundaryBoxMaxZ);
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
