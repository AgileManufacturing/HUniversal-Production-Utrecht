/**
 * @file ModelSpawner.h
 * @brief The library component of the model_spawner
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
	class ModelSpawner{
	protected:
		/**
		 * @param equipletName: The name of the equiplet to which this model spawner node belong to. 
		 * This will influence the advertisement paths of the services
		 * @param isShadow: Set to true if the equiplet being simulated is acting as the shadow (counter part) of the real equiplet. 
		 * This will influence the advertisement paths of the services
		 */
		ModelSpawner(std::string equipletName, bool isShadow);
	public:
		/**
		 * This method will spawn a model for a module by retrieving its data from the KDB, extracting it, parsing it, and calling a Gazebo service
		 * @param moduleIdentifier: the name of the module to spawn
		 */
		bool spawnModuleModel(rexos_datatypes::ModuleIdentifier moduleIdentifier);
		/**
		 * This method will spawn a model for an equiplet by retrieving its data from the KDB, extracting it, parsing it, and calling a Gazebo service
		 * @param gridPositionX: the target location of the model on the x axis in metres. The coordinate system matches the one used in REXOS
		 * @param gridPositionY: the target location of the model on the y axis in metres. The coordinate system matches the one used in REXOS
		 */
		bool spawnEquipletModel(double gridPositionX = 0, double gridPositionY = 0);
		/**
		 * This method will spawn a model for a part by retrieving its data from the KDB, extracting it, parsing it, and calling a Gazebo service
		 * @param positionX: the target location of the model on the x axis in metres. The coordinate system matches the one used in REXOS
		 * @param positionY: the target location of the model on the y axis in metres. The coordinate system matches the one used in REXOS
		 * @param positionZ: the target location of the model on the z axis in metres. The coordinate system matches the one used in REXOS
		 * @param positionX: the target location of the model on the x axis in metres. The coordinate system matches the one used in REXOS
		 * @param rotationX: the target rotation of the model on the x plane in radians. The coordinate system matches the one used in REXOS
		 * @param rotationY: the target rotation of the model on the x plane in radians. The coordinate system matches the one used in REXOS
		 * @param rotationZ: the target rotation of the model on the x plane in radians. The coordinate system matches the one used in REXOS
		 * @param spawnChildParts: If true, the method will recursively spawn any child parts (as defined by the KDB)
		 */
		bool spawnPartModel(std::string partName, rexos_datatypes::OriginPlacement originPlacement, 
				double positionX, double positionY, double positionZ, 
				double rotationX, double rotationY, double rotationZ, bool spawnChildParts = true);
		bool removeModuleModel(rexos_datatypes::ModuleIdentifier moduleIdentifier);
		bool removeEquipletModel();
		bool removePartModel(std::string partName);
	protected:
		/**
		 * This method will read the entire SDF file and return it as a string. The name of the SDF file is retrieved from the KDB
		 */
		std::string getSdfFileContents(rexos_knowledge_database::GazeboModel& gazeboModel, std::string uniqueName = "");
		/**
		 * This method will prepare a service call for adding a model in Gazebo and finally execute it.
		 * If the equiplet being simulated is a shadow, it will also add any relevant safety checks to the simulation
		 * @param model: A pointer to the model to be spawned. It must already be extracted
		 * @param modelName: The name of the model to be spawned. This must be a unique name and should not be reused (this triggers a bug in Gazebo)
		 * @param parentModel: A pointer to the model to which the models location and rotation is referenced and/or the model is attached to.
		 * If the model is not attached to anything and/or referenced to world, the pointer must be NULL.
		 * @param parentModelName: The name of the model to which the model to be spawned is referenced
		 * @param sdf: The content of the SDF file of the model to be spawned. This must already be parsed.
		 * @param pose: The location and rotation of the model to be spawned
		 * Example, for the parentModel "delta_robot_type_B" this would be "effector"
		 * @param robotNamespace: The namespace of the model to be spawned. This parameter is currently not used
		 */
		bool spawnModel(rexos_knowledge_database::GazeboModel* model, std::string& modelName, 
				std::string& sdf, geometry_msgs::Pose& pose, 
				rexos_knowledge_database::GazeboModel* parentModel = NULL, std::string parentModelName = "", 
				std::string robotNamespace = "");
		/**
		 * This method will prepare a service call for removing a model in Gazebo and finally execute it.
		 * If the equiplet being simulated is a shadow, it will also remove any relevant safety checks from the simulation
		 * @param model: A pointer to the model to be removed.
		 * @param parentModel: A pointer to the model to which the models location and rotation is referenced and/or the model is attached to.
		 * If the model is not attached to anything and/or referenced to world, the pointer must be NULL.
		 * @param modelName: The name of the model to be removed. 
		 */
		bool removeModel(rexos_knowledge_database::GazeboModel* model, std::string& modelName, 
				rexos_knowledge_database::GazeboModel* parentModel = NULL, std::string parentModelName = "");
	protected:
		std::string equipletName;
		bool isShadow;
		ros::NodeHandle nodeHandle;
		
		rexos_knowledge_database::Equiplet equiplet;
		
		/**
		 * This method will extract the gazebo model at the ZIP_ARCHIVE_PATH ("/tmp/rexos_model_spawner/") directory
		 * @param gazeboModel: The model to be extracted
		 * @param uniqueName: The unique name of the directory under which the model is extracted. 
		 * This directory is used to seperate different models of the same type. Multiple crates (GC4x4MB_x) may exist simultaneously in a simulation
		 * while they require their own unique QR codes. Overwriting these textures causes Gazebo to crash and thus they should be seperated.
		 */
		void extractGazeboModel(rexos_knowledge_database::GazeboModel& gazeboModel, std::string uniqueName = "");
	};
}
