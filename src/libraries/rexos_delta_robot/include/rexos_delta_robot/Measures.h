/**
 * @file Measures.h
 * @brief Miscellaneous measures.
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Daan Veltman
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

#pragma once
#include <rexos_utilities/Utilities.h>
#include <rexos_motor/CRD514KD.h>

namespace rexos_delta_robot{
	namespace Measures{
		/**
    	 * @var double BASE
    	 * Radius of the base in millimeters.
    	 **/
		const double BASE		= 101.3;

		/**
		 * @var double HIP
		 * Length of the hip in millimeters.
		 **/
		const double HIP		= 100;

		/**
		 * @var double EFFECTOR
		 * Radius of the effector in millimeters.
		 **/
		const double EFFECTOR	= 46.19;

		/**
		 * @var double ANKLE
		 * Length of the ankle in millimeters.
		 **/
		const double ANKLE		= 250;
		
		/**
		 * @var double HIP_ANKLE_ANGLE_MAX
		 * Maximum angle between hip and ankle when moving the ankle sideways in radians.
		 **/
		const double HIP_ANKLE_ANGLE_MAX = rexos_utilities::degreesToRadians(22);

		/**
		 * @var double MOTOR_ROT_MIN
		 * Mathematical minimum angle the motor is allowed to move to in radians.
		 **/
		const double MOTOR_ROT_MIN = rexos_utilities::degreesToRadians(-42);

		/**
		 * @var double MOTOR_ROT_MAX
		 * Mathematical maximum angle the motor is allowed to move to in radians.
		 **/
		const double MOTOR_ROT_MAX = rexos_utilities::degreesToRadians(55);
		
		/**
		 * @var double MOTORS_FROM_ZERO_TO_TOP_POSITION
		 * The angle from zero (horizontal) to the top position in radians.
		 **/
		const double MOTORS_FROM_ZERO_TO_TOP_POSITION = rexos_utilities::degreesToRadians(42.5); 

		/**
		 * @var double BOUNDARY_BOX_MAX_X
		 * The max x-coordinate of the effector boundary box in millimeters.
		 **/
 		const double BOUNDARY_BOX_MAX_X = 500;

 		/**
		 * @var double BOUNDARY_BOX_MAX_Y
		 * The max y-coordinate of the effector boundary box in millimeters.
		 **/
		const double BOUNDARY_BOX_MAX_Y = BOUNDARY_BOX_MAX_X;

		/**
		 * @var double BOUNDARY_BOX_MIN_X
		 * The min x-coordinate of the effector boundary box in millimeters.
		 **/
		const double BOUNDARY_BOX_MIN_X = -BOUNDARY_BOX_MAX_X;

		/**
		 * @var double BOUNDARY_BOX_MIN_Y
		 * The min y-coordinate of the effector boundary box in millimeters.
		 **/
		const double BOUNDARY_BOX_MIN_Y = -BOUNDARY_BOX_MAX_Y;

		/**
		 * @var double BOUNDARY_BOX_MIN_Z
		 * The min z-coordinate of the effector boundary box in millimeters.
		 **/
		const double BOUNDARY_BOX_MIN_Z = -250;

		/**
		 * @var double BOUNDARY_BOX_MAX_Z
		 * The max z-coordinate of the effector boundary box in millimeters.
		 **/
		const double BOUNDARY_BOX_MAX_Z = -180;

		/**
		 * @var double CALIBRATION_STEP_SMALL
		 * The size of the small steps in the calibration in radians.
		 **/
		const double CALIBRATION_STEP_SMALL = rexos_motor::CRD514KD::MOTOR_STEP_ANGLE;
		
		/**
		 * @var double CALIBRATION_STEP_BIG
		 * The size of the big calibration steps in radians. Currently equal to 20 small calibration steps.
		 **/
		 const double CALIBRATION_STEP_BIG = CALIBRATION_STEP_SMALL * 20;
	}
}
