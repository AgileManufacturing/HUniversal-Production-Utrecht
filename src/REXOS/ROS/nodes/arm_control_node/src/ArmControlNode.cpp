#include "ros/ros.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/GetLinkState.h"
#include "arm_control_node/ArmControlNode.h"
#include "arm_control_node/MoveCalculations.h"

#include <sstream>

ArmControlNode::ArmControlNode()
{
	
}



void ArmControlNode::run(int argc, char **argv){
	ros::start();
	ros::Rate loop_rate(10);
	MoveCalculations moveCalculations;
	moveCalculations.moveEffectorToPoint(90,90,-350,7.5,7.5,7.5);

	ROS_INFO("=====arm_control_node is running======");
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_effort_client");
	ArmControlNode armControlNode;
	armControlNode.run(argc, argv);
	return 0;
}
