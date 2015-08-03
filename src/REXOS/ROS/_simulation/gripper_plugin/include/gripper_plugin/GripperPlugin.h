/**
 * @file GripperPlugin.h
 * @brief The gripper plugin is used to allow grippers to dynamically and fully automatically attach to other models (collisions). 
 * The grippers can be activated by calling a service. This service will be advertised as a modbus command and advertised at address specified in the SDF file.
 * The gripper plugin is designed to be used with the @link rexos_io library
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

#include <rexos_io/writeU16.h>
#include <rexos_io/readU16.h>
#include <gripper_plugin/Gripper.h>

#include <vector>

using namespace gazebo;

namespace gripper_plugin {
	class GripperPlugin : public ModelPlugin {
	public: 
		GripperPlugin();
		~GripperPlugin();
	public: 
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		/**
		 * This method is called just BEFORE the world in gazebo is updated
		 */
		void onUpdate(const common::UpdateInfo & info);
		
		bool writeGripperActive(rexos_io::writeU16::Request& request, 
				rexos_io::writeU16::Response& response);
		bool readGripperActive(rexos_io::readU16::Request& request, 
				rexos_io::readU16::Response& response);
	protected:
		ros::NodeHandle* nodeHandle;
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		
		ros::ServiceServer writeGripperActiveServer;
		ros::ServiceServer readGripperActiveServer;
		
		std::vector<Gripper> grippers;
		
		transport::NodePtr node;
	private:
		transport::SubscriberPtr contactSub;
		
		void onContacts(ConstContactsPtr& msg);
	};
	GZ_REGISTER_MODEL_PLUGIN(gripper_plugin::GripperPlugin)
}
