/**
 * @file AttachPlugin.h
 * @brief The collision plugin is a safety plugin used for monitoring collisions (contacts) between models.
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
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Collision.hh>
#include <ros/ros.h>

#include <collision_plugin/Collision.h>
#include <collision_plugin/addCollision.h>
#include <collision_plugin/removeCollision.h>
#include <collision_plugin/addContactExclusion.h>
#include <collision_plugin/removeContactExclusion.h>
#include <vector>

using namespace gazebo;

namespace collision_plugin {
	/**
	 * @struct ContactExclusion
	 * A contact exclusion will exclode any collisions (contacts) between models (collisions) despite the forces exposed.
	 */
	struct ContactExclusion {
		physics::CollisionPtr collision1;
		physics::CollisionPtr collision2;
	};
	
	class CollisionPlugin : public WorldPlugin {
	public: 
		CollisionPlugin();
		~CollisionPlugin();
	public: 
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		/**
		 * This method is called just BEFORE the world in gazebo is updated
		 */
		void onUpdate(const common::UpdateInfo & info);
		
	protected:
		ros::NodeHandle* nodeHandle;
		physics::WorldPtr world;
		event::ConnectionPtr updateConnection;
		
		ros::ServiceServer addCollisionServer;
		ros::ServiceServer removeCollisionServer;
		ros::ServiceServer addContactExclusionServer;
		ros::ServiceServer removeContactExclusionServer;
		
		std::vector<Collision> collisions;
		std::vector<ContactExclusion> exclusions;
		
		transport::NodePtr node;
	private:
		transport::SubscriberPtr contactSub;
		
		/**
		 * This method is called whenever the physics engine publishes a contact on the topic. This method runs async.
		 */
		void onContacts(ConstContactsPtr& msg);
		
		bool addCollision(collision_plugin::addCollision::Request& request, collision_plugin::addCollision::Response& response);
		bool removeCollision(collision_plugin::removeCollision::Request& request, collision_plugin::removeCollision::Response& response);
		bool addContactExclusion(collision_plugin::addContactExclusion::Request& request, collision_plugin::addContactExclusion::Response& response);
		bool removeContactExclusion(collision_plugin::removeContactExclusion::Request& request, collision_plugin::removeContactExclusion::Response& response);
	};
	GZ_REGISTER_WORLD_PLUGIN(collision_plugin::CollisionPlugin)
}
