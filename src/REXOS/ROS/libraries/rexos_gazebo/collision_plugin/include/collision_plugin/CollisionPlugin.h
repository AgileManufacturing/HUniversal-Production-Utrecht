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
		
		void onContacts(ConstContactsPtr& msg);
		
		bool addCollision(collision_plugin::addCollision::Request& request, collision_plugin::addCollision::Response& response);
		bool removeCollision(collision_plugin::removeCollision::Request& request, collision_plugin::removeCollision::Response& response);
		bool addContactExclusion(collision_plugin::addContactExclusion::Request& request, collision_plugin::addContactExclusion::Response& response);
		bool removeContactExclusion(collision_plugin::removeContactExclusion::Request& request, collision_plugin::removeContactExclusion::Response& response);
	};
	GZ_REGISTER_WORLD_PLUGIN(collision_plugin::CollisionPlugin)
}
