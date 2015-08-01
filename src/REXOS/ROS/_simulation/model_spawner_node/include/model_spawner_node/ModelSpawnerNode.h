/**
 * @file ModelSpawnerNode.h
 * @brief The ROS node component of the model_spawner. 
 * This allows REXOS to interface with gazebo by adding and removing models of modules, equiplets and parts.
 * @date Created: 2015-06-00
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

#include "ros/ros.h"
#include <string>
#include <vector>

#include "rexos_logger/rexos_logger.h"
#include <model_spawner_node/spawnModule.h>
#include <model_spawner_node/removeModule.h>
#include <model_spawner_node/spawnPart.h>
#include <model_spawner_node/removePart.h>
#include <rexos_model_spawner/ModelSpawner.h>

namespace model_spawner_node {
	class ModelSpawnerNode : public rexos_model_spawner::ModelSpawner {
	public:
		/**
		 * @param equipletName: The name of the equiplet to which this model spawner node belong to. 
		 * This will influence the advertisement paths of the services
		 * @param isShadow: Set to true if the equiplet being simulated is acting as the shadow (counter part) of the real equiplet. 
		 * This will influence the advertisement paths of the services
		 */
		ModelSpawnerNode(std::string equipletName, bool isShadow);
		
	private:
		/**
		 * Service callback for <equipletName>/model/spawnModule
		 */
		bool spawnModule(spawnModule::Request &request, spawnModule::Response &response);
		/**
		 * Service callback for <equipletName>/model/removeModule
		 */
		bool removeModule(removeModule::Request &request, removeModule::Response &response);
		/**
		 * Service callback for <equipletName>/model/spawnPart
		 */
		bool spawnPart(spawnPart::Request &request, spawnPart::Response &response);
		/**
		 * Service callback for <equipletName>/model/removePart
		 */
		bool removePart(removePart::Request &request, removePart::Response &response);
		
		std::string equipletName;
		
		ros::ServiceServer spawnModelServer;
		ros::ServiceServer removeModelServer;
		ros::ServiceServer spawnPartServer;
		ros::ServiceServer removePartServer;
	};
}
