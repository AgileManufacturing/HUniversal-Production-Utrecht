/**
 * @file StepperMotorProperties.cpp
 * @brief Contains the properties of a stepper motor.
 * @date Created: 2012-10-02
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

#define ZIP_ARCHIVE_PATH "/tmp/rexos_model_spawner/"
#define QR_CODE_FILENAME "qrCode.png"

#include <rexos_knowledge_database/rexos_knowledge_database.h>
#include <rexos_knowledge_database/GazeboModel.h>
#include <rexos_knowledge_database/Equiplet.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_datatypes/OriginPlacement.h>
#include "rexos_logger/rexos_logger.h"

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

namespace rexos_model_spawner{
	enum OriginPlacementType {
		RELATIVE_TO_EQUIPLET_ORIGIN,
		RELATIVE_TO_MODULE_ORIGIN,
		RELATIVE_TO_WORLD_ORIGIN
	};
	
	class ModelSpawner{
	protected:
		ModelSpawner(std::string equipletName, bool isShadow);
	public:
		void spawnModuleModel(rexos_datatypes::ModuleIdentifier moduleIdentifier);
		void spawnEquipletModel(double gridPositionX = 0, double gridPositionY = 0);
		void spawnPartModel(std::string partName, rexos_datatypes::OriginPlacement originPlacement, 
				double positionX, double positionY, double positionZ, 
				double rotationX, double rotationY, double rotationZ, bool spawnChildParts = true);
		void removeModuleModel(rexos_datatypes::ModuleIdentifier moduleIdentifier);
		void removeEquipletModel();
		void removePartModel(std::string partName);
	protected:
		std::string getSdfFileContents(rexos_knowledge_database::GazeboModel& gazeboModel, std::string uniqueName = "");
		void spawnModel(rexos_knowledge_database::GazeboModel* model, rexos_knowledge_database::GazeboModel* parentModel, 
				std::string& modelName, std::string& sdf, geometry_msgs::Pose& pose, 
				std::string referenceModel = "", std::string referenceLink = "", std::string robotNamespace = "");
		void removeModel(std::string modelName);
	protected:
		std::string equipletName;
		bool isShadow;
		ros::NodeHandle nodeHandle;
		
		rexos_knowledge_database::Equiplet equiplet;
		
		void extractGazeboModel(rexos_knowledge_database::GazeboModel& gazeboModel, std::string uniqueName = "");
	};
}
