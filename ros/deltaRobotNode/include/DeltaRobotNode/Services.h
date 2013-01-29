/**
 * @file DeltaRobotNode/Services.h
 * @brief Names for the DeltaRobot services.
 * @date Created: 2012-10-04
 *
 * @author Dick van der Steen
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

#include <string>

namespace DeltaRobotNodeServices{

	// Old, deprecated services -----------------------------------------------
	/**
	 * @var const std::string MOVE_TO_POINT
	 * Name for the service in which a deltarobot moves to a specific coordinate.
	 **/
	const std::string MOVE_TO_POINT = "moveToPoint";
	
	/**
	 * @var const std::string MOVE_PATH
	 * Name for the service in which a deltarobot moves along a path of specific coordinates.
	 **/
	const std::string MOVE_PATH = "movePath";
	
	/**
	 * @var const std::string MOVE_TO_RELATIVE_POINT
	 * Name for the service in which a deltarobot moves to a coordinate relative to the current coordinate.
	 **/
	const std::string MOVE_TO_RELATIVE_POINT = "moveToRelativePoint";
	
	/**
	 * @var const std::string MOVE_RELATIVE_PATH
	 * Name for the service in which a deltarobot moves along a path of coordinates relative to the preceding coordinate.
	 **/
	const std::string MOVE_RELATIVE_PATH = "moveRelativePath";
	
	/**
	 * @var const std::string CALIBRATE
	 * Name for the service in which a deltarobot calibrates itself.
	 **/
	const std::string CALIBRATE = "calibrate";

	// json services ----------------------------------------------------------
	/**
	 * @var const std::string MOVE_TO_POINT_JSON
	 * Name for the service in which a deltarobot moves to a specific coordinate.
	 **/
	const std::string MOVE_TO_POINT_JSON = "DeltaRobotNode/moveToPoint";

	/**
	 * @var const std::string MOVE_PATH_JSON
	 * Name for the service in which a deltarobot moves along a path of specific coordinates.
	 **/
	const std::string MOVE_PATH_JSON = "DeltaRobotNode/movePath";

	/**
	 * @var const std::string MOVE_TO_RELATIVE_POINT_JSON
	 * Name for the service in which a deltarobot moves to a coordinate relative to the current coordinate.
	 **/
	const std::string MOVE_TO_RELATIVE_POINT_JSON = "DeltaRobotNode/moveToRelativePoint";

	/**
	 * @var const std::string MOVE_RELATIVE_PATH_JSON
	 * Name for the service in which a deltarobot moves along a path of coordinates relative to the preceding coordinate.
	 **/
	const std::string MOVE_RELATIVE_PATH_JSON = "DeltaRobotNode/moveRelativePath";

	/**
	 * @var const std::string CALIBRATE_JSON
	 * Name for the service in which a deltarobot calibrates itself.
	 **/
	const std::string CALIBRATE_JSON = "DeltaRobotNode/calibrate";
}
