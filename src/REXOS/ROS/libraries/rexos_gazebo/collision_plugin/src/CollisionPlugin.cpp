#include "collision_plugin/CollisionPlugin.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <ros/callback_queue.h>
#include <boost/algorithm/string.hpp>
#include <map>
#include <rexos_logger/rexos_logger.h>

using namespace gazebo;

namespace collision_plugin {
	CollisionPlugin::CollisionPlugin() : WorldPlugin(), contactSub(NULL) {
		std::cout << "CollisionPlugin constructed" << std::endl;
	}
	CollisionPlugin::~CollisionPlugin() {
	}
	void CollisionPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		std::cout << "CollisionPlugin loading" << std::endl;
		
		world = _world;
		
		// Subscribing to contact topics
		node = transport::NodePtr(new transport::Node());
		node->Init(world->GetName());
		
		/*physics::ContactManager* mgr = model->GetWorld()->GetPhysicsEngine()->GetContactManager();
		std::string topic = mgr->CreateFilter(model->GetName(), collisions);*/
		std::string topic = "~/physics/contacts";
		std::cout << "Subcribing to contact topic " << topic << std::endl;
		contactSub = node->Subscribe(topic, &CollisionPlugin::onContacts, this);
		
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
		addCollisionServer = nodeHandle->advertiseService(rosNamespace + "/collision/addCollision/", 
				&CollisionPlugin::addCollision, this);
		removeCollisionServer = nodeHandle->advertiseService(rosNamespace + "/collision/removeCollision/", 
				&CollisionPlugin::removeCollision, this);
		addContactExclusionServer = nodeHandle->advertiseService(rosNamespace + "/collision/addContactExclusion/", 
				&CollisionPlugin::addContactExclusion, this);
		removeContactExclusionServer = nodeHandle->advertiseService(rosNamespace + "/collision/removeContactExclusion/", 
				&CollisionPlugin::removeContactExclusion, this);
		
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&CollisionPlugin::onUpdate, this, _1));
	}
	void CollisionPlugin::onUpdate(const common::UpdateInfo& info) {
		common::Time currTime = world->GetSimTime();
		for(uint i = 0; i < collisions.size(); i++) {
			collisions[i].handleUpdate(currTime);
		}
	}
	void CollisionPlugin::onContacts(ConstContactsPtr& msg) {
		for (int i = 0; i < msg->contact_size(); i++) {
			physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
				world->GetEntity(msg->contact(i).collision1()));
			physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
				world->GetEntity(msg->contact(i).collision2()));
			
			// is contact excluded?
			bool isContactExcluded = false;
			for(uint j = 0; j < exclusions.size(); j++) {
				if((exclusions[j].collision1 == collision1 && exclusions[j].collision2 == collision2) || 
						(exclusions[j].collision1 == collision2 && exclusions[j].collision2 == collision1)) {
					isContactExcluded = true;
				}
			}
			
			if(isContactExcluded == false) {
				for(uint j = 0; j < collisions.size(); j++) {
					physics::CollisionPtr collision = collisions[j].getCollision();
				
					if(collision1 == collision) {
						collisions[j].handleContact(msg->contact(i), world, 1);
					} else if (collision2 == collision) {
						collisions[j].handleContact(msg->contact(i), world, 2);
					}
				}
			}
		}
	}
	bool CollisionPlugin::addCollision(collision_plugin::addCollision::Request& request, 
			collision_plugin::addCollision::Response& response) {
		REXOS_INFO_STREAM("Adding collision " << request.model << "::" << request.link << "::" << request.collision);
		physics::ModelPtr modelPtr = world->GetModel(request.model);
		if(modelPtr == NULL) return false;
		physics::LinkPtr linkPtr = modelPtr->GetLink(request.link);
		if(linkPtr == NULL) return false;
		physics::CollisionPtr collisionPtr = linkPtr->GetCollision(request.collision);
		if(collisionPtr == NULL) return false;
		
		Collision collision(collisionPtr, request.maxForce, request.maxTorque, nodeHandle);
		collisions.push_back(collision);
		return true;
	}
	bool CollisionPlugin::removeCollision(collision_plugin::removeCollision::Request& request, 
			collision_plugin::removeCollision::Response& response) {
		REXOS_INFO_STREAM("Removing collision " << request.model << "::" << request.link << "::" << request.collision);
		physics::ModelPtr modelPtr = world->GetModel(request.model);
		if(modelPtr == NULL) return false;
		physics::LinkPtr linkPtr = modelPtr->GetLink(request.link);
		if(linkPtr == NULL) return false;
		physics::CollisionPtr collisionPtr = linkPtr->GetCollision(request.collision);
		if(collisionPtr == NULL) return false;
		
		for(auto it = collisions.begin(); it < collisions.end(); it++) {
			if(collisionPtr == it->getCollision()) {
				collisions.erase(it);
				return true;
			}
		}
		return false;
	}
	bool CollisionPlugin::addContactExclusion(collision_plugin::addContactExclusion::Request& request, 
			collision_plugin::addContactExclusion::Response& response) {
		REXOS_INFO_STREAM("Adding contactExclusion " << request.model1 << "::" << request.link1 << "::" << request.collision1 << "\n"
				<< request.model2 << "::" << request.link2 << "::" << request.collision2);
		physics::ModelPtr modelPtr1 = world->GetModel(request.model1);
		if(modelPtr1 == NULL) return false;
		physics::LinkPtr linkPtr1 = modelPtr1->GetLink(request.link1);
		if(linkPtr1 == NULL) return false;
		physics::CollisionPtr collisionPtr1 = linkPtr1->GetCollision(request.collision1);
		if(collisionPtr1 == NULL) return false;
		
		physics::ModelPtr modelPtr2 = world->GetModel(request.model2);
		if(modelPtr2 == NULL) return false;
		physics::LinkPtr linkPtr2 = modelPtr2->GetLink(request.link2);
		if(linkPtr2 == NULL) return false;
		physics::CollisionPtr collisionPtr2 = linkPtr2->GetCollision(request.collision2);
		if(collisionPtr2 == NULL) return false;
		
		// does the exclusion already exist?
		for(auto it = exclusions.begin(); it < exclusions.end(); it++) {
			if((collisionPtr1 == it->collision1 && collisionPtr2 == it->collision2) ||
				(collisionPtr1 == it->collision2 && collisionPtr2 == it->collision1)) {
				return false;
			}
		}
		
		// add it
		ContactExclusion exclusion;
		exclusion.collision1 = collisionPtr1;
		exclusion.collision2 = collisionPtr2;
		exclusions.push_back(exclusion);
		return true;
	}
	bool CollisionPlugin::removeContactExclusion(collision_plugin::removeContactExclusion::Request& request, 
			collision_plugin::removeContactExclusion::Response& response) {
		REXOS_INFO_STREAM("Removing contactExclusion " << request.model1 << "::" << request.link1 << "::" << request.collision1 << "\n"
				<< request.model2 << "::" << request.link2 << "::" << request.collision2);
		physics::ModelPtr modelPtr1 = world->GetModel(request.model1);
		if(modelPtr1 == NULL) return false;
		physics::LinkPtr linkPtr1 = modelPtr1->GetLink(request.link1);
		if(linkPtr1 == NULL) return false;
		physics::CollisionPtr collisionPtr1 = linkPtr1->GetCollision(request.collision1);
		if(collisionPtr1 == NULL) return false;
		
		physics::ModelPtr modelPtr2 = world->GetModel(request.model2);
		if(modelPtr2 == NULL) return false;
		physics::LinkPtr linkPtr2 = modelPtr2->GetLink(request.link2);
		if(linkPtr2 == NULL) return false;
		physics::CollisionPtr collisionPtr2 = linkPtr2->GetCollision(request.collision2);
		if(collisionPtr2 == NULL) return false;
		
		for(auto it = exclusions.begin(); it < exclusions.end(); it++) {
			if((collisionPtr1 == it->collision1 && collisionPtr2 == it->collision2) ||
				(collisionPtr1 == it->collision2 && collisionPtr2 == it->collision1)) {
				exclusions.erase(it);
				return true;
			}
		}
		return false;
	}
}
