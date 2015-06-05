#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <ros/ros.h>

#include <acceleration_plugin/Entity.h>
#include <acceleration_plugin/addEntity.h>
#include <acceleration_plugin/removeEntity.h>
#include <vector>

using namespace gazebo;

namespace acceleration_plugin {
	class AccelerationPlugin : public WorldPlugin {
	public: 
		AccelerationPlugin();
		~AccelerationPlugin();
	public: 
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		void onPreUpdate(const common::UpdateInfo & info);
		void onPostUpdate();
	protected:
		ros::NodeHandle* nodeHandle;
		event::ConnectionPtr preUpdateConnection;
		event::ConnectionPtr postUpdateConnection;
		
		ros::ServiceClient violationOccuredClient;
		ros::ServiceServer addEntityServer;
		ros::ServiceServer removeEntityServer;
		
		std::vector<Entity> entities;
	private:
		physics::WorldPtr world;
		
		bool addEntity(acceleration_plugin::addEntity::Request& request, acceleration_plugin::addEntity::Response& response);
		bool removeEntity(acceleration_plugin::removeEntity::Request& request, acceleration_plugin::removeEntity::Response& response);
	};
	GZ_REGISTER_WORLD_PLUGIN(acceleration_plugin::AccelerationPlugin)
}
