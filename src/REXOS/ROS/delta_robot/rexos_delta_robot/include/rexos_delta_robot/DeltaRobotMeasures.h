/**
 * @file DeltaRobotMeasures.h
 * @brief Holds all measures for the DeltaRobot
 * @date Created: 2012-10-03
 *
 * @author Koen Braham
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

#pragma once

#include <jsoncpp/json/value.h>

namespace rexos_delta_robot{
	/**
	 * A class that holds all measures for the DeltaRobot.
	 **/
	class DeltaRobotMeasures{
	public:
		/**
		 * @var double base
		 * Radius of the base in millimeters.
		 **/
		double base;

		/**
		 * @var double hip
		 * Length of the hip in millimeters.
		 **/
		double hip;

		/**
		 * @var double effector
		 * Radius of the effector in millimeters.
		 **/
		double effector; 

		/**
		 * @var double ankle
		 * Length of the ankle in millimeters.
		 **/
		double ankle; 

		/**
		 * @var double maxAngleHipAnkle
		 * Maximum angle between hip and ankle when moving the ankle sideways.
		 **/
		double maxAngleHipAnkle;
		
		/**
		 * @var double boundaryBoxMinX
		 * The max x-coordinate of the effector boundary box in millimeters.
		 **/
		double boundaryBoxMinX;
 		/**
		 * @var double boundaryBoxMaxX
		 * The max y-coordinate of the effector boundary box in millimeters.
		 **/
		double boundaryBoxMaxX;
		/**
		 * @var double BOUNDARY_BOX_MIN_X
		 * The min x-coordinate of the effector boundary box in millimeters.
		 **/
		double boundaryBoxMinY;
		/**
		 * @var double BOUNDARY_BOX_MIN_Y
		 * The min y-coordinate of the effector boundary box in millimeters.
		 **/
		double boundaryBoxMaxY;
		/**
		 * @var double BOUNDARY_BOX_MIN_Z
		 * The min z-coordinate of the effector boundary box in millimeters.
		 **/
		double boundaryBoxMinZ;
		/**
		 * @var double BOUNDARY_BOX_MAX_Z
		 * The max z-coordinate of the effector boundary box in millimeters.
		 **/
		double boundaryBoxMaxZ;
		
		
		DeltaRobotMeasures(Json::Value properties);
	};
}
