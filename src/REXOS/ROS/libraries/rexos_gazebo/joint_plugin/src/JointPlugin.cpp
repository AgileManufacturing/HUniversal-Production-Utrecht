#include "joint_plugin/JointPlugin.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <ros/callback_queue.h>
#include <boost/algorithm/string.hpp>
#include <map>

using namespace gazebo;

namespace joint_plugin {
	JointPlugin::JointPlugin() : WorldPlugin() {
		std::cout << "JointPlugin constructed" << std::endl;
	}
	JointPlugin::~JointPlugin() {
	}
	void JointPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		std::cout << "JointPlugin loading" << std::endl;
		
		world = _world;
		
		// Advertising ROS services
		if(_sdf->HasElement("rosNamespace") == false) {
			std::cerr << "Missing rosNamespace" << std::endl;
			return;
		}
		std::string rosNamespace = _sdf->GetElement("rosNamespace")->GetValue()->GetAsString();
		ROS_INFO_STREAM("Advertising services at " << rosNamespace);
		
		if (!ros::isInitialized()) {
			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}
		
		nodeHandle = new ros::NodeHandle();
		addJointServer = nodeHandle->advertiseService(rosNamespace + "/joint/addJoint", 
				&JointPlugin::addJoint, this);
		removeJointServer = nodeHandle->advertiseService(rosNamespace + "/joint/removeJoint", 
				&JointPlugin::removeJoint, this);
		
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&JointPlugin::onUpdate, this, _1));
	}
	void JointPlugin::onUpdate(const common::UpdateInfo& info) {
		common::Time currTime = world->GetSimTime();
		for(uint i = 0; i < joints.size(); i++) {
			joints[i].handleUpdate(currTime);
		}
	}
	
	bool JointPlugin::addJoint(joint_plugin::addJoint::Request& request, 
			joint_plugin::addJoint::Response& response) {
		ROS_INFO_STREAM("Adding joint " << request.model << "::" << request.joint);
		physics::ModelPtr model = world->GetModel(request.model);
		if(model == NULL) return false;
		physics::JointPtr joint = model->GetJoint(request.joint);
		if(joint == NULL) return false;
		
		Joint Joint(joint, request.maxErrorDistance, nodeHandle);
		joints.push_back(Joint);
		return true;
	}
	bool JointPlugin::removeJoint(joint_plugin::removeJoint::Request& request, 
			joint_plugin::removeJoint::Response& response) {
		ROS_INFO_STREAM("Removing joint " << request.model << "::" << request.joint);
		for(auto it = joints.begin(); it < joints.end(); it++) {
			joints.erase(it);
			return true;
		}
		return false;
	}
}
