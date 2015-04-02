/**
 * @file StewartGoughNode.h
 * @brief Names for the StewartGoughNode services.
 * @date Created: 2012-10-17
 *
 * @author Garik Hakopian
 *
 * @section LICENSE
 * License: newBSD
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

#ifndef STEWARTGOUGH_H
#define STEWARTGOUGH_H

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"

#include <rexos_datatypes/EquipletStep.h>
#include <rexos_utilities/Utilities.h>
#include <rexos_stewart_gough/StewartGough.h>
#include <rexos_stewart_gough/StewartGoughLocation.h>
#include <rexos_motor/StepperMotor.h>
#include <rexos_motor/StepperMotorProperties.h>
#include <rexos_module/ActorModule.h>

#include <jsoncpp/json/value.h>

namespace stewart_gough_node {
	/**
	 * the DeltaRobotNode which is a ModuleStateMachine
	**/
	class StewartGoughNode : public rexos_module::ActorModule {
	protected:
		rexos_motor::StepperMotorProperties* stepperMotorProperties;
		rexos_stewart_gough::StewartGoughMeasures* stewartGoughMeasures;
	
		std::string modbusIp;
		int modbusPort;
		
		int calibrationBigStepFactor;
		
		
	public:
		StewartGoughNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, bool isShadow);
		virtual ~StewartGoughNode();
		
		virtual bool transitionInitialize();
		virtual bool transitionDeinitialize();
		virtual bool transitionSetup();
		virtual bool transitionShutdown();
		virtual bool transitionStart();
		virtual bool transitionStop();
		
		// Main functions to be called from the services
		bool calibrate();
		bool moveToPoint(rexos_stewart_gough::StewartGoughLocation to, double maxAcceleration);
		
		void onSetInstruction(const rexos_module::SetInstructionGoalConstPtr &goal);

	private:
		float lastX;
		float lastY;
		float lastZ;
		/**
		 * @var DeltaRobot::DeltaRobot * deltaRobot
		 * the deltaRobot
		 **/
		rexos_stewart_gough::StewartGough * stewartGough;
	};
}
#endif
