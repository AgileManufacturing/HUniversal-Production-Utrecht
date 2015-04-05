/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
 *
 * @author Dennis Koole
 * @author Alexander Streng
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

#include <unistd.h>
#include <model_spawner_node/ModelSpawnerNode.h>
#include <rexos_knowledge_database/Equiplet.h>

#include "ros/ros.h"

using namespace model_spawner_node;

/**
 * Create a new EquipletNode
 * @param id The unique identifier of the Equiplet
 **/
ModelSpawnerNode::ModelSpawnerNode(std::string equipletName, bool spawnEquipletModel, bool isShadow) :
		rexos_model_spawner::ModelSpawner(equipletName, isShadow), 
		equipletName(equipletName)
{
	if(spawnEquipletModel == true) {
		REXOS_INFO("spawning equiplet model");
		ModelSpawner::spawnEquipletModel();
	}
	
	spawnNodeServer = nodeHandle.advertiseService(equipletName + "/spawnModel", &ModelSpawnerNode::spawnModel, this);
	
	REXOS_INFO("model_spawner_node has been started");
}

bool ModelSpawnerNode::spawnModel(spawnModel::Request &request, spawnModel::Response &response) {
	rexos_datatypes::ModuleIdentifier identifier(request.manufacturer, request.typeNumber, request.serialNumber);
	ModelSpawner::spawnModuleModel(identifier);
	return true;
}
