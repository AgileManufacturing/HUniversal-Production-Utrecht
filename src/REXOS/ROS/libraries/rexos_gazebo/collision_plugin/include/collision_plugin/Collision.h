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

namespace collision_plugin {
	class Collision {
	public:
		Collision();
		Collision(physics::CollisionPtr collision, double maxForce, double maxTorque, ros::NodeHandle* nodeHandle);
		Collision(const Collision& rhs);
		Collision operator=(const Collision& rhs);
		
		void handleContact(const msgs::Contact& contact, const physics::WorldPtr world, int collisionIndex);
		void handleUpdate(common::Time currentTime);
		
		physics::CollisionPtr getCollision();
	protected:
		physics::CollisionPtr collision;
		double maxForce;
		double maxTorque;
		
		boost::mutex m;
		std::map<physics::CollisionPtr, common::Time> collisionTimes;
		std::map<physics::CollisionPtr, int> collisionContactCount;
		
		ros::Publisher violationPublisher;
	};
}
