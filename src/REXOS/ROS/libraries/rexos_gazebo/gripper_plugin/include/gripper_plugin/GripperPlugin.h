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
