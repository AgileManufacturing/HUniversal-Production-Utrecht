/**
 * @file InverseKinematics.h
 * @brief Inverse kinematics implementation. Based on work from Viacheslav Slavinsky.\n
 * conventions sitting in front of delta robot:\n
 * x-axis goes from left to right\n
 * y-axis goes from front to back\n
 * z-axis goes from bottom to top\n
 * point (0,0,0) lies in the middle of all the motors at the motor's height
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

#include <rexos_datatypes/Point3D.h>
#include <rexos_datatypes/MotorRotation.h>
#include <rexos_datatypes/DeltaRobotMeasures.h>
#include <rexos_delta_robot/InverseKinematicsModel.h>

namespace rexos_delta_robot{
	/**
	 * Inverse kinematics implementation. Based on work from Viacheslav Slavinsky\n
	 * Conventions sitting in front of delta robot:\n
	 * x-axis goes from left to right\n
	 * y-axis goes from front to back\n
	 * z-axis goes from bottom to top\n
	 * point (0,0,0) lies in the middle of all the motors at the motor's height
	 **/
	class InverseKinematics : public InverseKinematicsModel{
	private:
		double motorAngle(const rexos_datatypes::Point3D<double>& destinationPoint,
				double motorLocation) const;

	public:
		InverseKinematics(const double base, const double hip,
			const double effector, const double ankle,
			const double maxAngleHipAnkle);

		InverseKinematics(rexos_datatypes::DeltaRobotMeasures& deltaRobotMeasures);

		virtual ~InverseKinematics(void);
		
		void destinationPointToMotorRotations(const rexos_datatypes::Point3D<double>& destinationPoint,
				rexos_datatypes::MotorRotation* (&rotations)[3]) const;
	};
}
