//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           measures.h
// Description:    miscellaneous measures
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          -
//
// License:        GNU GPL v3
//
// This file is part of huniplacer.
//
// huniplacer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// huniplacer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with huniplacer.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#pragma once
#include <huniplacer/utils.h>

namespace huniplacer
{
	namespace measures
	{
		/* Robot v1 
		
		const double BASE 				  = 101.3; //mm
		const double HIP 				  = 61.1;  //mm
		const double EFFECTOR 			  = 46.19; //mm old value: 42.15
		const double ANKLE				  = 150.8; //mm
		*/
		
		/* Robot v2 */
		const double BASE 				  = 101.3; //mm
		const double HIP 				  = 100; //mm
		const double EFFECTOR			  = 46.19; //mm
		const double ANKLE 				  = 250; //mm
		
		const double HIP_ANKLE_ANGLE_MAX  = utils::rad(22);    //radians
		//safety constants, roughly determined to be as safe as possible for testing purposes
		const double MOTOR_ROT_MIN 	      = utils::rad(-42);   //radians
		const double MOTOR_ROT_MAX 	      = utils::rad(45);    //radians

		const double MOTOR1_DEVIATION	  = utils::rad(-45);   //radians
		const double MOTOR2_DEVIATION	  = utils::rad(-45);   //radians
		const double MOTOR3_DEVIATION	  = utils::rad(-45);   //radians

		/* Robot v1 
		//make sure all the points that the effector can reach are included in the box
		// with the following dimensions:
		const double MAX_X = 85;
		const double MAX_Y = MAX_X;
		const double MIN_X = -MAX_X;
		const double MIN_Y = -MAX_Y;
		const double MIN_Z = -210;
		const double MAX_Z = -100;
		*/
		
		/* Robot v2 */
		//make sure all the points that the effector can reach are included in the box
		// with the following dimensions:
		const double MAX_X = 150;
		const double MAX_Y = MAX_X;
		const double MIN_X = -MAX_X;
		const double MIN_Y = -MAX_Y;
		//safety constants, roughly determined to be as safe as possible for testing purposes
		const double MIN_Z = -280;
		const double MAX_Z = -200;
	}
}
