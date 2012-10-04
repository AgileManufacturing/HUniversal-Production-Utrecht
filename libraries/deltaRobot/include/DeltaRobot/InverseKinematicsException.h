//******************************************************************************
//
//                 REXOS
//
//******************************************************************************
// Project:        	DeltaRobot
// File:           	InverseKinematicsException.h
// Description:    	exception thrown if an invalid value is calculated during inverse kinematics
// Author:         	1.0 Lukas Vermond & Kasper van Nieuwland
// 					1.1 Daan Veltman
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

#include <stdexcept>

#include <DataTypes/MotorRotation.h>
#include <DataTypes/Point3D.h>

namespace DeltaRobot {
/**
 * Inverse Kinematics -> Instances of deltarobot will throw this exception whenever they fail to convert a point.
 **/
class InverseKinematicsException: public std::runtime_error {
private:
	DataTypes::Point3D<double> notConvertablePoint;

public:
	InverseKinematicsException(const char* exceptionMessage,
			DataTypes::Point3D<double> destinationPoint) :
			std::runtime_error(exceptionMessage), notConvertablePoint(
					destinationPoint) {
	}

	virtual ~InverseKinematicsException(void) throw () {
	}

	/**
	 * Used to access the point that could not be converted
	 **/
	DataTypes::Point3D<double> getNotConvertablePoint(void) {
		return notConvertablePoint;
	}
};
}
