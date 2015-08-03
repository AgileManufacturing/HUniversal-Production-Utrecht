/**
 * @file AccelerationPlugin.h
 * @brief The acceleration plugin is a safety plugin used for monitoring the acceleration of various links of models
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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <ros/ros.h>

#include <acceleration_plugin/Entity.h>
#include <acceleration_plugin/addEntity.h>
#include <acceleration_plugin/removeEntity.h>
#include <vector>

using namespace gazebo;

namespace acceleration_plugin {
	class AccelerationPlugin : public WorldPlugin {
	public: 
		AccelerationPlugin();
		~AccelerationPlugin();
	public: 
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		/**
		 * This method is called just BEFORE the world in gazebo is updated
		 */
		void onPreUpdate(const common::UpdateInfo & info);
		/**
		 * This method is called just AFTER the world in gazebo is updated
		 */
		void onPostUpdate();
	protected:
		ros::NodeHandle* nodeHandle;
		event::ConnectionPtr preUpdateConnection;
		event::ConnectionPtr postUpdateConnection;
		
		ros::ServiceClient violationOccuredClient;
		ros::ServiceServer addEntityServer;
		ros::ServiceServer removeEntityServer;
		
		std::vector<Entity> entities;
	private:
		physics::WorldPtr world;
		
		/**
		 * Service callback for <rosNamespace>/acceleration/addEntity
		 */
		bool addEntity(acceleration_plugin::addEntity::Request& request, acceleration_plugin::addEntity::Response& response);
		/**
		 * Service callback for <rosNamespace>/acceleration/removeEntity
		 */
		bool removeEntity(acceleration_plugin::removeEntity::Request& request, acceleration_plugin::removeEntity::Response& response);
	};
	GZ_REGISTER_WORLD_PLUGIN(acceleration_plugin::AccelerationPlugin)
}
