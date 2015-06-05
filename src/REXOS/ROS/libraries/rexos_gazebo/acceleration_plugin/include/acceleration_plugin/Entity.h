#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Time.hh>
#include <ros/ros.h>

#include <vector>

using namespace gazebo;

namespace acceleration_plugin {
	class Entity {
	public:
		Entity();
		Entity(physics::LinkPtr link, double maxAcceleration, common::Time currentTime, ros::NodeHandle* nodeHandle);
		
		void handleUpdate(common::Time currentTime);
		
		physics::LinkPtr getLink();
	protected:
		physics::LinkPtr link;
		double maxAcceleration;
		ros::Publisher violationPublisher;
	private:
		double previousVelocity;
		common::Time previousTime;
	};
}
