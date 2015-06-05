#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <ros/ros.h>

#include <vector>

using namespace gazebo;

namespace joint_plugin {
	class Joint {
	public:
		Joint();
		Joint(physics::JointPtr joint, double maxErrorPose, ros::NodeHandle* nodeHandle);
		
		void handleUpdate(common::Time currentTime);
	protected:
		physics::JointPtr joint;
		double maxErrorPose;
		ros::Publisher violationPublisher;
	};
}
