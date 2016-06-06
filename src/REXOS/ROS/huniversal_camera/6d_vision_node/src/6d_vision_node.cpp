#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr & msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "6d_vision_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("6d_vision_node_info", 1000, chatterCallback);

	ros::spin();

	return 0;
}
