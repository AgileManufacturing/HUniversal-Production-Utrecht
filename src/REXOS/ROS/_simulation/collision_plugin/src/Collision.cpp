#include "collision_plugin/Collision.h"

#include <gazebo/physics/PhysicsEngine.hh>
#include <std_msgs/String.h>
#include <rexos_logger/rexos_logger.h>

using namespace gazebo;

namespace collision_plugin {
	Collision::Collision() : 
		collision(NULL), maxForce(0), maxTorque(0) {
	}
	Collision::Collision(physics::CollisionPtr collision, double maxForce, double maxTorque, ros::NodeHandle* nodeHandle) : 
			collision(collision), maxForce(maxForce), maxTorque(maxTorque) {
		violationPublisher = nodeHandle->advertise<std_msgs::String>("collision/violation/", 5);
	}
	Collision::Collision(const Collision& rhs) : 
			collision(rhs.collision), maxForce(rhs.maxForce), maxTorque(rhs.maxTorque), violationPublisher(rhs.violationPublisher) {
	}
	Collision Collision::operator=(const Collision& rhs) {
		return Collision(rhs);
	}
	
	physics::CollisionPtr Collision::getCollision() {
		return collision;
	}
	void Collision::handleContact(const msgs::Contact& contact, const physics::WorldPtr world, int collisionIndex) {
		boost::mutex::scoped_lock lock(m);
		
		double maxExposedForce = 0;
		double maxExposedTorque = 0;
		
		for(int i = 0; i < contact.position_size(); i++) {
			math::Vector3 force;
			math::Vector3 torque;
			if(collisionIndex == 1) {
				force = math::Vector3(
					contact.wrench(i).body_1_wrench().force().x(),
					contact.wrench(i).body_1_wrench().force().y(),
					contact.wrench(i).body_1_wrench().force().z());
				torque = math::Vector3(
					contact.wrench(i).body_1_wrench().torque().x(),
					contact.wrench(i).body_1_wrench().torque().y(),
					contact.wrench(i).body_1_wrench().torque().z());
			} else {
				force = math::Vector3(
					contact.wrench(i).body_2_wrench().force().x(),
					contact.wrench(i).body_2_wrench().force().y(),
					contact.wrench(i).body_2_wrench().force().z());
				torque = math::Vector3(
					contact.wrench(i).body_2_wrench().torque().x(),
					contact.wrench(i).body_2_wrench().torque().y(),
					contact.wrench(i).body_2_wrench().torque().z());
			}
			
			if(force.GetLength() > maxExposedForce) {
				maxExposedForce = force.GetLength();
			}
			if(torque.GetLength() > maxExposedTorque) {
				maxExposedTorque = torque.GetLength();
			}
		}
		
		if(maxExposedForce > maxForce || maxExposedTorque > maxTorque) {
			REXOS_INFO_STREAM("Illigal contact between " << contact.collision1() << " AND " << contact.collision2()
					<< " EXPOSED_FORCE " << maxExposedForce << " EXPOSED_TORQUE " << maxExposedTorque
					<< " FORCE " << maxForce << " TORQUE " << maxTorque);
			
			std_msgs::String message;
			if(collisionIndex == 1) {
				message.data = contact.collision1();
			} else {
				message.data = contact.collision2();
			}
			violationPublisher.publish(message);
		}
	}
	void Collision::handleUpdate(common::Time currentTime) {
		boost::mutex::scoped_lock lock(m);
		
	}
}
