#include "joint_plugin/Joint.h"

#include <std_msgs/String.h>

using namespace gazebo;

namespace joint_plugin {
	Joint::Joint() : 
		joint(NULL), maxErrorPose(0) {
	}
	Joint::Joint(physics::JointPtr joint, double maxErrorPose, ros::NodeHandle* nodeHandle) : 
			joint(joint), maxErrorPose(maxErrorPose) {
		violationPublisher = nodeHandle->advertise<std_msgs::String>("joint/violation/", 5);
	}
	void Joint::handleUpdate(common::Time currentTime) {
		math::Pose errorPose = joint->GetAnchorErrorPose();
		math::Vector3 errorPositionVector = errorPose.pos;
		double errorPosition = errorPositionVector.GetLength();
		
		if(std::abs(errorPosition) > maxErrorPose) {
			ROS_INFO_STREAM("Exceeded joint limitations: " << 
					joint->GetParent()->GetModel()->GetName() << "::" << joint->GetName() << 
					" went " << errorPosition << " instead of " << maxErrorPose);
			std_msgs::String message;
			message.data = joint->GetParent()->GetModel()->GetName();
			violationPublisher.publish(message);
		}
	}
}
