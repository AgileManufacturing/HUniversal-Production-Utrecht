//******************************************************************************
//
//                 REXOS
//
//******************************************************************************
// Project:        DeltaRobot
// File:           Measures.h
// Description:    miscellaneous measures
// Author:         1.0 Lukas Vermond & Kasper van Nieuwland
// 				   1.1 Koen Braham		Daan Veltman
// Notes:          -
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
#include <Utilities/Utilities.h>

/**
 * measures.h -> deltarobot measures, specifically for the huniplacer deltarobot
 **/

namespace DeltaRobot
{
	namespace Measures
	{
		const double BASE 				  = 101.3; //mm
		const double HIP 				  = 100; //mm
		const double EFFECTOR			  = 46.19; //mm
		const double ANKLE 				  = 250; //mm
		
		const double HIP_ANKLE_ANGLE_MAX  = Utilities::rad(22);    //radians
		//safety constants, roughly determined to be as safe as possible for testing purposes
		const double MOTOR_ROT_MIN 	      = Utilities::rad(-42);   //radians
		const double MOTOR_ROT_MAX 	      = Utilities::rad(45);    //radians

		const double MOTOR1_DEVIATION	  = Utilities::rad(-45);   //radians
		const double MOTOR2_DEVIATION	  = Utilities::rad(-45);   //radians
		const double MOTOR3_DEVIATION	  = Utilities::rad(-45);   //radians

		// Top (granite) to middle point is 45 degrees. Removing the hip thickness results in +-42.5 degrees!
		const double MOTORS_DEVIATION	=   Utilities::rad(42.5); 
		
 		const double MAX_X = 500;
		const double MAX_Y = MAX_X;
		const double MIN_X = -MAX_X;
		const double MIN_Y = -MAX_Y;
		const double MIN_Z = -250;
		const double MAX_Z = -180;
	}
}
