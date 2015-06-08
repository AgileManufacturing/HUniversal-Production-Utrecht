#include "acceleration_plugin/Entity.h"

#include <std_msgs/String.h>

using namespace gazebo;

namespace acceleration_plugin {
	Entity::Entity() : 
		link(NULL), maxAcceleration(0) {
	}
	Entity::Entity(physics::LinkPtr link, double maxAcceleration, common::Time currentTime, ros::NodeHandle* nodeHandle) : 
			link(link), maxAcceleration(maxAcceleration), numberOfIterations(0) {
		violationPublisher = nodeHandle->advertise<std_msgs::String>("acceleration/violation/", 5);
	}
	physics::LinkPtr Entity::getLink() {
		return link;
	}
	
	void Entity::handleUpdate(common::Time currentTime) {
		math::Vector3 currentPosition = link->GetWorldCoGPose().pos;
		
		// safeguard against using invalid data
		if(numberOfIterations >= NUMBER_OF_SAMPLES) {
			math::Vector3 totalAcceleration;
			for(int i = 0; i < NUMBER_OF_SAMPLES - 2; i++) {
				common::Time firstTimeDifference = times[i + 1] - times[i];
				common::Time secondTimeDifference = times[i + 2] - times[i + 1];
				double elapsedTime = (times[i + 1].Double() + secondTimeDifference.Double() / 2)
						- (times[i].Double() + firstTimeDifference.Double() / 2);
				
				math::Vector3 firstPositionDifferenceVector = positions[i + 1] - positions[i];
				math::Vector3 secondPositionDifferenceVector = positions[i + 2] - positions[i + 1];
				math::Vector3 firstVelocityVector = firstPositionDifferenceVector / firstTimeDifference.Double();
				math::Vector3 secondVelocityVector = secondPositionDifferenceVector / secondTimeDifference.Double();
				math::Vector3 velocityDifferenceVector = secondVelocityVector - firstVelocityVector;
				
				math::Vector3 currentAcceleration = velocityDifferenceVector / elapsedTime;
				totalAcceleration += currentAcceleration;
			}
			double acceleration = totalAcceleration.GetLength() / (NUMBER_OF_SAMPLES - 2);
			
			if(std::abs(acceleration) > maxAcceleration) {
				ROS_INFO_STREAM("Exceeded acceleration limitations: " << 
						link->GetModel()->GetName() << "::" << link->GetName() << 
						" went " << acceleration << " instead of " << maxAcceleration);
				
				std_msgs::String message;
				message.data = link->GetModel()->GetName();
				violationPublisher.publish(message);
			}
		} else {
			numberOfIterations++;
		}
		
		for(int i = 0; i < NUMBER_OF_SAMPLES - 1; i++) {
			positions[i] = positions[i + 1];
			times[i] = times[i + 1];
		}
		positions[NUMBER_OF_SAMPLES - 1] = currentPosition;
		times[NUMBER_OF_SAMPLES - 1] = currentTime;
		
		
		/*
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
		previousTime = currentTime;*/
	}
}
