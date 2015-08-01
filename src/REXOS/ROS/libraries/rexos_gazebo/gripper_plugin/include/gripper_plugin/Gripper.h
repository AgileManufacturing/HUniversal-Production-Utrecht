/**
 * @file Gripper.h
 * @brief Grippers describe the actual contact point of a gripper. It will determine when the gripper actually attaches (determined by two parameters)
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
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <ros/ros.h>

#include <rexos_io/writeU16.h>

#include <vector>
#include <map>

#define MIN_CONTACT_DURATION 0.0
#define MIN_CONTACT_POINTS 1

using namespace gazebo;

namespace gripper_plugin {
	class Gripper {
	public:
		Gripper();
		Gripper(physics::LinkPtr link, physics::CollisionPtr collision);
		Gripper(const Gripper& rhs);
		
		void setActive(bool active);
		bool getIsActive();
		/**
		 * This method is called whenever the physics engine publishes a contact on the topic. This method runs async.
		 * This method will store any contact in the two maps for later processing
		 */
		void handleContact(const msgs::Contact& contact, const physics::WorldPtr world);
		/**
		 * This method is called just BEFORE the world in gazebo is updated
		 * This method will read the content of the two maps and determine if the contacts satisfy the parameters
		 */
		void handleUpdate(common::Time currentTime);
		
		physics::CollisionPtr getCollision();
	protected:
		physics::LinkPtr link;
		physics::CollisionPtr collision;
		physics::JointPtr attachJoint;
		bool isActive;
		bool isAttached;
		
		boost::mutex m;
		/**
		 * This map will store the time of the first collision (contact) with the other model (collision)
		 */
		std::map<physics::CollisionPtr, common::Time> collisionTimes;
		/**
		 * This map will store the number of collisions (contact)s with the other model (collision)
		 */
		std::map<physics::CollisionPtr, int> collisionContactCount;
		/**
		 * This method will attach the gripper to the model (collision) specified by reinitializing the joint.
		 * It will also reset the limitations of the joint to 0, resulting in a fixed joint.
		 */
		void attachToCollision(physics::CollisionPtr collision);
		void detach();
	};
}
