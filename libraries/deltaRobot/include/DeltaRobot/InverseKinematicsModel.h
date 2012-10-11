/**
 * @file InverseKinematicsModel.h
 * @brief Abstract inverse kinematics model for use in DeltaRobot class.
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Daan Veltman
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

#include <DataTypes/Point3D.h>
#include <DataTypes/MotorRotation.h>

namespace DeltaRobot {
	/**
	 * InverseKinematicsModel.h -> kinematics model of the deltarobot.
	 *
	 * Uses various lengths and sizes to calculate a point to a motion.
	 *
	 *  conventions:
	 *	sitting in front of delta robot
	 *	x-axis goes from left to right
	 *	y-axis goes from front to back
	 *	z-axis goes from bottom to top
	 *	point (0,0,0) lies in the middle of all the motors at the motor's height
	 **/
	class InverseKinematicsModel {
	protected:
		/**
		 * Radius of the base in millimeters
		 **/ 
		const double base;

		/*
		 * Length of the hip in millimeters
		 **/
		const double hip;

		/**
		 * Radius of the effector in millimeters
		 **/ 
		const double effector;

		/**
		 * Length of the ankle in millimeters
		 **/
		const double ankle;

		InverseKinematicsModel(const double base, const double hip,
				const double effector, const double ankle,
				const double maxAngleHipAnkle) :
				base(base), hip(hip), effector(effector), ankle(ankle), maxAngleHipAnkle(
						maxAngleHipAnkle) {
		}

	public:
		virtual ~InverseKinematicsModel(void) {
		}

		/** 
		 * Maximum angle between hip and ankle on x-z plane in radians.
		 **/
		const double maxAngleHipAnkle;

		/**
		 * @brief converts a point to a motion
		 * @param destinationPoint point that shall be converted
		 * @param motionPointer output parameter, the results of the conversion will be stored here
		 **/
		virtual void pointToMotion(const DataTypes::Point3D<double>& destinationPoint,
				DataTypes::MotorRotation<double>* (&rotations)[3]) const = 0;

	};
}
