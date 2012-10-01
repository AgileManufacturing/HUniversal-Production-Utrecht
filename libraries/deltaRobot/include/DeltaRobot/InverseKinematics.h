//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        	huniplacer
// File:           	inverseKinematics.h
// Description:    	inverse kinematics implementation. Based on work from Viacheslav Slavinsky
// Author:         	Lukas Vermond & Kasper van Nieuwland
// Edited by:		Daan Veltman
// Notes:          	-
//
// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#pragma once

#include <DataTypes/Point3D.h>
#include <DataTypes/motion.h>
#include <huniplacer/InverseKinematicsModel.h>

namespace DeltaRobot {
	class InverseKinematics: public InverseKinematicsModel {
	private:
		double motorAngle(const Point3D& destinationPoint,
				double motorLocation) const;

	public:
		InverseKinematics(const double base, const double hip,
				const double effector, const double ankle,
				const double hip_ankle_angle_max);

		virtual ~InverseKinematics(void);
		
		void pointToMotion(const Point3D& destinationPoint,
				motionf& motionPointer) const;
	};
}
