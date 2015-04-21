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

using namespace gazebo;

namespace gripper_plugin {
	class Gripper {
	public:
		Gripper();
		Gripper(physics::LinkPtr link, physics::CollisionPtr collision);
		Gripper(const Gripper& rhs);
		
		void setActive(bool active);
		bool getIsActive();
		void handleContact(const msgs::Contact& contact, const physics::WorldPtr world);
		void handleUpdate(common::Time currentTime);
		
		physics::CollisionPtr getCollision();
	protected:
		physics::LinkPtr link;
		physics::CollisionPtr collision;
		physics::JointPtr attachJoint;
		bool isActive;
		bool isAttached;
		
		boost::mutex m;
		std::map<physics::CollisionPtr, common::Time> collisionTimes;
		std::map<physics::CollisionPtr, int> collisionContactCount;
		
		void attachToCollision(physics::CollisionPtr collision);
		void detach();
	};
}
