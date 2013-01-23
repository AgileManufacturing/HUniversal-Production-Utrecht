/**
 * @file InverseKinematicsException.h
 * @brief Exception thrown if an invalid value is calculated during inverse kinematics.
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

#include <stdexcept>

#include <rexos_datatypes/MotorRotation.h>
#include <rexos_datatypes/Point3D.h>

namespace rexos_delta_robot{
	/**
	 * Exception thrown if an invalid value is calculated during inverse kinematics.
	 **/
	class InverseKinematicsException: public std::runtime_error{
	private:
		/**
		 * @var Point3D<double> notConvertablePoint
		 * Point that could not be converted into angles for the motors.
		 **/
		rexos_datatypes::Point3D<double> notConvertablePoint;

	public:
		/**
		 * Constructor for InverseKinematicsException.
		 * @param exceptionMessage The exception message
		 * @param destinationPoint The point of the destination.
		 **/
		InverseKinematicsException(const char* exceptionMessage, rexos_datatypes::Point3D<double> destinationPoint) :
				std::runtime_error(exceptionMessage), notConvertablePoint(destinationPoint){}

		/**
		 * Destructor for InverseKinematicsException.
		 **/
		virtual ~InverseKinematicsException(void) throw(){}

		/**
		 * Used to access the point that could not be converted.
		 * @return A point3D<double> of the point that could not be converted.
		 **/
		rexos_datatypes::Point3D<double> getNotConvertablePoint(void){
			return notConvertablePoint;
		}
	};
}
