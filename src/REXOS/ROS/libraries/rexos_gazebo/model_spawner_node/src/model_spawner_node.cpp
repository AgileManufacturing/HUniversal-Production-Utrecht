/**
 * @file EquipletNode.cpp
 * @brief Main for EquipletNode
 * @date Created: 2012-10-12
 *
 * @author Joris Vergeer
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

#include <model_spawner_node/ModelSpawnerNode.h>

int main(int argc, char **argv) {
	if (argc < 2) {
		REXOS_ERROR("Usage: model_spawner_node (--isShadow) (--spawnEquipletModel) equipletName");
		return -1;
	}
	
	std::string command;
	for(int i = 0; i < argc; i++) {
		command += std::string(argv[i]) + " ";
	}
	REXOS_INFO_STREAM("command " << command);
	
	
	bool spawnEquipletModel = false;
	bool spawnModuleModels = false;
	bool spawnPartModel = false;
	bool isShadow = false;
	
	int positionX = 0;
	int positionY = 0;
	int positionZ = 0;
	double rotationX = 0;
	double rotationY = 0;
	double rotationZ = 0;
	
	rexos_datatypes::OriginPlacement originPlacement;
	std::string partName;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--spawnEquipletModel") {
			spawnEquipletModel = true;
		} else if (arg == "--spawnModuleModels") {
			spawnModuleModels = true;
		} else if (arg == "--spawnPartModel") {
			spawnPartModel = true;
		} else if (arg == "-x") {
			positionX = boost::lexical_cast<int>(argv[++i]);
			REXOS_INFO_STREAM("positionX " << positionZ);
		} else if (arg == "-y") {
			positionY = boost::lexical_cast<int>(argv[++i]);
			REXOS_INFO_STREAM("positionY " << positionY);
		} else if (arg == "-z") {
			positionZ = boost::lexical_cast<int>(argv[++i]);
			REXOS_INFO_STREAM("positionZ " << positionZ);
		} else if (arg == "--pitch") {
			rotationX = boost::lexical_cast<double>(argv[++i]);
			REXOS_INFO_STREAM("rotationX " << rotationX);
		} else if (arg == "--roll") {
			rotationY = boost::lexical_cast<double>(argv[++i]);
			REXOS_INFO_STREAM("rotationY " << rotationY);
		} else if (arg == "--yaw") {
			rotationZ = boost::lexical_cast<double>(argv[++i]);
			REXOS_INFO_STREAM("rotationZ " << rotationZ);
		} else if (arg == "--partName") {
			partName = argv[++i];
		} else if (arg == "--originPlacementType") {
			originPlacement.setOriginPlacementType(argv[++i]);
			if(originPlacement.getOriginPlacementType() != rexos_datatypes::OriginPlacement::RELATIVE_TO_WORLD_ORIGIN) {
				// we need the identifier
				Json::Value originPlacementParameters;
				originPlacementParameters["relativeTo"] = argv[++i];
				originPlacement.setParameters(originPlacementParameters);
			}
		} else if (arg == "--isShadow") {
			isShadow = true;
		}
	}
	
	// TODO this is quite a nasty hack
	std::string equipletName;
	if(spawnPartModel == false) {
		equipletName = std::string(argv[argc - 1]);
	} else {
		equipletName = "NOT_RELEVANT";
	}
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_modelSpawner";
	ros::init(argc, argv, nodeName);
	
	model_spawner_node::ModelSpawnerNode modelSpawnerNode(equipletName, isShadow);
	
	if(spawnEquipletModel == true) {
		REXOS_INFO("spawning equiplet model");
		modelSpawnerNode.spawnEquipletModel(positionX, positionY);
	}
	
	if(spawnModuleModels == true) {
		rexos_knowledge_database::Equiplet equiplet(equipletName);
		std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiers = equiplet.getModuleIdentifiersOfAttachedModules();
		
		for(uint i = 0; i < moduleIdentifiers.size(); i++) {
			modelSpawnerNode.spawnModuleModel(moduleIdentifiers[i]);
		}
		ros::shutdown();
	}
	if(spawnPartModel == true) {
		modelSpawnerNode.spawnPartModel(partName, originPlacement, positionX, positionY, positionZ, rotationX, rotationY, rotationZ, true);
		ros::shutdown();
	}
	
	ros::spin();
	return 0;
}
