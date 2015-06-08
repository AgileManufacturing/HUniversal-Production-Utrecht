#include "gripper_plugin/Gripper.h"

#include <gazebo/physics/PhysicsEngine.hh>

using namespace gazebo;

namespace gripper_plugin {
	Gripper::Gripper() : 
		link(NULL), collision(NULL) {
	}
	Gripper::Gripper(physics::LinkPtr link, physics::CollisionPtr collision) : 
			link(link), collision(collision), isActive(false) {
		physics::PhysicsEnginePtr physicsEngine = link->GetWorld()->GetPhysicsEngine();
		attachJoint = physicsEngine->CreateJoint("revolute", link->GetModel());
	}
	Gripper::Gripper(const Gripper& rhs) : 
			link(rhs.link), collision(rhs.collision), attachJoint(rhs.attachJoint), isActive(rhs.isActive), isAttached(rhs.isAttached), 
			collisionTimes(rhs.collisionTimes), collisionContactCount(rhs.collisionContactCount) {
	}
	void Gripper::setActive(bool active) {
		boost::mutex::scoped_lock lock(m);
		isActive = active;
		if(isActive == false && attachJoint->GetChild() != NULL) {
			detach();
		}
	}
	bool Gripper::getIsActive() {
		return isActive;
	}
	
	void Gripper::handleContact(const msgs::Contact& contact, const physics::WorldPtr world) {
		boost::mutex::scoped_lock lock(m);
		
		if(isActive == true && isAttached == false) {
			physics::CollisionPtr collision1 = boost::dynamic_pointer_cast<physics::Collision>(
				world->GetEntity(contact.collision1()));
			physics::CollisionPtr collision2 = boost::dynamic_pointer_cast<physics::Collision>(
				world->GetEntity(contact.collision2()));
			
			physics::CollisionPtr collisionOfOtherObject;
			if(collision1 == this->collision) collisionOfOtherObject = collision2;
			else if(collision2 == this->collision) collisionOfOtherObject = collision1;
			else throw std::runtime_error("Recieved contact which does not involve this gripper");
			
			if(collisionTimes.find(collisionOfOtherObject) == collisionTimes.end()) {
				// first time we collided with this one
				collisionTimes[collisionOfOtherObject] = msgs::Convert(contact.time());
			}
			// keep track of number of contacts per update iteration
			collisionContactCount[collisionOfOtherObject] += 1;
		}
	}
	void Gripper::handleUpdate(common::Time currentTime) {
		boost::mutex::scoped_lock lock(m);
		
		if(isActive == true && isAttached == false) {
			auto it = collisionTimes.begin();
			while(it != collisionTimes.end()) {
				if(collisionContactCount.find(it->first) == collisionContactCount.end()) {
					// no contacts with the collision were recorded previous iteration, remove it
					it = collisionTimes.erase(it);
					// do not increment it, as erase points it to the next position
				} else if(currentTime - it->second >= 0.0 && collisionContactCount[it->first] >= 1) {
					attachToCollision(it->first);
					break;
				} else {
					it++;
				}
			}
		}
		collisionContactCount.empty();
	}
	physics::CollisionPtr Gripper::getCollision() {
		return collision;
	}
	void Gripper::attachToCollision(physics::CollisionPtr collision) {
		std::cout << "Attaching to " << collision->GetLink()->GetModel()->GetName() << "::" << collision->GetLink()->GetName() << std::endl;
		
		attachJoint->Load(link, collision->GetLink(), math::Pose());
		attachJoint->Init();
		attachJoint->SetHighStop(0, 0);
		attachJoint->SetLowStop(0, 0);
		
		isAttached = true;
	}
	void Gripper::detach() {
		std::cout << "Detaching" << std::endl;
		attachJoint->Detach();
		isAttached = false;
	}
}
