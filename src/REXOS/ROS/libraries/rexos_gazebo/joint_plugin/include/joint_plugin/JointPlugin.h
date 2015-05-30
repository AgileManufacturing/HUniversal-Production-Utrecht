#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <ros/ros.h>

#include <joint_plugin/Joint.h>
#include <joint_plugin/addJoint.h>
#include <joint_plugin/removeJoint.h>
#include <vector>

using namespace gazebo;

namespace joint_plugin {
	class JointPlugin : public WorldPlugin {
	public: 
		JointPlugin();
		~JointPlugin();
	public: 
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		void onUpdate(const common::UpdateInfo & info);
		
	protected:
		ros::NodeHandle* nodeHandle;
		event::ConnectionPtr updateConnection;
		
		ros::ServiceClient violationOccuredClient;
		ros::ServiceServer addJointServer;
		ros::ServiceServer removeJointServer;
		
		std::vector<Joint> entities;
	private:
		physics::WorldPtr world;
		
		bool addJoint(joint_plugin::addJoint::Request& request, joint_plugin::addJoint::Response& response);
		bool removeJoint(joint_plugin::removeJoint::Request& request, joint_plugin::removeJoint::Response& response);
	};
	GZ_REGISTER_WORLD_PLUGIN(joint_plugin::JointPlugin)
}
