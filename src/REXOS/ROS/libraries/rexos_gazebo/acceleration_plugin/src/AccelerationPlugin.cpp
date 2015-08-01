#include "acceleration_plugin/AccelerationPlugin.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <rexos_logger/rexos_logger.h>
#include <ros/callback_queue.h>
#include <boost/algorithm/string.hpp>
#include <map>

using namespace gazebo;

namespace acceleration_plugin {
	AccelerationPlugin::AccelerationPlugin() : WorldPlugin() {
		std::cout << "AccelerationPlugin constructed" << std::endl;
	}
	AccelerationPlugin::~AccelerationPlugin() {
	}
	void AccelerationPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		std::cout << "AccelerationPlugin loading" << std::endl;
		
		world = _world;
		
		// Advertising ROS services
		if(_sdf->HasElement("rosNamespace") == false) {
			std::cerr << "Missing rosNamespace" << std::endl;
			return;
		}
		std::string rosNamespace = _sdf->GetElement("rosNamespace")->GetValue()->GetAsString();
		REXOS_INFO_STREAM("Advertising services at " << rosNamespace);
		
		if (!ros::isInitialized()) {
			REXOS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}
		
		nodeHandle = new ros::NodeHandle();
		addEntityServer = nodeHandle->advertiseService(rosNamespace + "/acceleration/addEntity", 
				&AccelerationPlugin::addEntity, this);
		removeEntityServer = nodeHandle->advertiseService(rosNamespace + "/acceleration/removeEntity", 
				&AccelerationPlugin::removeEntity, this);
		
		this->preUpdateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&AccelerationPlugin::onPreUpdate, this, _1));
		this->postUpdateConnection = event::Events::ConnectWorldUpdateEnd(
			boost::bind(&AccelerationPlugin::onPostUpdate, this));
	}
	
	void AccelerationPlugin::onPreUpdate(const common::UpdateInfo& info) {
		// nothing to do
	}
	void AccelerationPlugin::onPostUpdate() {
		common::Time currTime = world->GetSimTime();
		for(uint i = 0; i < entities.size(); i++) {
			entities[i].handleUpdate(currTime);
		}
	}
	
	bool AccelerationPlugin::addEntity(acceleration_plugin::addEntity::Request& request, 
			acceleration_plugin::addEntity::Response& response) {
		REXOS_INFO_STREAM("Adding entity " << request.model << "::" << request.link);
		physics::ModelPtr model = world->GetModel(request.model);
		if(model == NULL) return false;
		physics::LinkPtr link = model->GetLink(request.link);
		if(link == NULL) return false;
		
		Entity entity(link, request.maxAcceleration, world->GetSimTime(), nodeHandle);
		entities.push_back(entity);
		return true;
	}
	bool AccelerationPlugin::removeEntity(acceleration_plugin::removeEntity::Request& request, 
			acceleration_plugin::removeEntity::Response& response) {
		REXOS_INFO_STREAM("Removing entity " << request.model << "::" << request.link);
		physics::ModelPtr model = world->GetModel(request.model);
		if(model == NULL) return false;
		physics::LinkPtr link = model->GetLink(request.link);
		if(link == NULL) return false;
		
		for(auto it = entities.begin(); it < entities.end(); it++) {
			if(link == it->getLink()) {
				entities.erase(it);
				return true;
			}
		}
		return false;
	}
}
