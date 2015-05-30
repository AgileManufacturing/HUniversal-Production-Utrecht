#include "acceleration_plugin/Entity.h"

#include <std_msgs/String.h>

using namespace gazebo;

namespace acceleration_plugin {
	Entity::Entity() : 
		link(NULL), maxAcceleration(0) {
	}
	Entity::Entity(physics::LinkPtr link, double maxAcceleration, common::Time currentTime, ros::NodeHandle* nodeHandle) : 
			link(link), maxAcceleration(maxAcceleration), previousTime(currentTime) {
		violationPublisher = nodeHandle->advertise<std_msgs::String>("acceleration/violation/", 5);
	}
	physics::LinkPtr Entity::getLink() {
		return link;
	}
	
	void Entity::handleUpdate(common::Time currentTime) {
		math::Vector3 velocityVector = link->GetWorldLinearVel();
		double velocity = velocityVector.GetLength();
		double velocityDifference = velocity - previousVelocity;
		common::Time timeDifference = currentTime - previousTime;
		if(timeDifference == 0) {
			// apparently the time did not proceed (this happens when constructing)
			return;
		}
		
		double acceleration = velocityDifference / timeDifference.Double();
		
		if(std::abs(acceleration) > maxAcceleration) {
			ROS_INFO_STREAM("Exceeded acceleration limitations: " << 
					link->GetModel()->GetName() << "::" << link->GetName() << 
					" went " << acceleration << " instead of " << maxAcceleration);
			std_msgs::String message;
			message.data = link->GetModel()->GetName();
			violationPublisher.publish(message);
		}
		
		previousVelocity = velocity;
		previousTime = currentTime;
	}
}
