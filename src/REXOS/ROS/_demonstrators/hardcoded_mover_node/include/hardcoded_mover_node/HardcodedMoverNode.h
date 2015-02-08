/**
 * @file HardcodedMoverNode.h
 * @brief An ROS node for moving the delta robot with hardcoded movements
 * @date Created: 2015-02-08
 *
 * @author Tommas Bakker
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

#include <ros/ros.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_datatypes/OriginPlacement.h>
#include <rexos_module/ModuleInterface.h>
#include "rexos_logger/rexos_logger.h"
#include <vectors/Vectors.h>

namespace hardcoded_mover_node {

	class HardcodedMoverNode : public rexos_module::ModuleInterfaceListener
	{
	public:
		struct Movement {
			rexos_datatypes::OriginPlacement originPlacement;
			double moveX, moveY, moveZ;
			double rotateX, rotateY, rotateZ;
			double acceleration;
			Movement() : moveX(0), moveY(0), moveZ(0), 
				rotateX(0), rotateY(0), rotateZ(0), 
				acceleration(0)
			{
				// nothing to do
			}
		};
	public:
		HardcodedMoverNode(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, std::vector<Movement> movements, bool loop = false);
		~HardcodedMoverNode();
		virtual void onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, std::string id, bool completed);
		void startNextMovement(string id);
		
	private:
		std::string equipletName;
		rexos_datatypes::ModuleIdentifier identifier;
		bool loop;
		std::vector<Movement> movements;
		std::vector<Movement>::iterator movementIterator;
		rexos_module::ModuleInterface moduleInterface;
	};

}
