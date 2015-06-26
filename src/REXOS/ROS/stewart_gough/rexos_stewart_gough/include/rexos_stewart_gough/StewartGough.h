/**
 * @file StewartGough.h
 * @brief Symbolizes an entire StewartGough.
 *
 * @author Garik Hakopian
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

#include <rexos_motorized_actor/MotorizedActor.h>
#include <rexos_stewart_gough/StewartGoughMeasures.h>
#include <rexos_stewart_gough/StewartGoughLocation.h>
#include <rexos_stewart_gough/SixAxisCalculations.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <jsoncpp/json/value.h>
#include "rexos_logger/rexos_logger.h"

#include <vector>

namespace rexos_stewart_gough {
	struct MotorGroup {
		int motorIndex1;
		int motorIndex2;
		
		MotorGroup(int motorIndex1 = 0, int motorIndex2 = 0):
				motorIndex1(motorIndex1),
				motorIndex2(motorIndex2){}
	};
	
	/**
	 * A class that symbolizes an entire deltarobot.
	 **/
	class StewartGough : public rexos_motorized_actor::MotorizedActor {
	public:
		StewartGough(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, Json::Value node);
		virtual ~StewartGough();
		
		void readJSONNode(Json::Value node);
		
		bool checkPath(const StewartGoughLocation& begin, const StewartGoughLocation& end);
		void moveTo(StewartGoughLocation point, double maxAcceleration);
		
		virtual bool calibrateMotors();
		
		StewartGoughLocation getEffectorLocation();
		
	private:
		
		//SixAxisCalculations * sixAxisCalculations;
		
		rexos_stewart_gough::StewartGoughMeasures stewartGoughMeasures;
		
		SixAxisCalculations* sixAxisCalculations;
		
		
		/**
		 * @var Point3D<double> effectorLocation
		 * A 3D point in doubles that points to the location of the effector.
		 **/
		StewartGoughLocation effectorLocation;
		
		double getSpeedForRotation(double relativeAngle, double moveTime, double acceleration);
		double getAccelerationForRotation(double relativeAngle, double moveTime);
	};
}
