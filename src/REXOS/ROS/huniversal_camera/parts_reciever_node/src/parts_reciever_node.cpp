#include "ros/ros.h"
#include "parts_reciever_node/PartPro.h"
#include "rexos_logger/rexos_logger.h"
#include <string>
#include <sstream>

void chatterCallback(const boost::shared_ptr<parts_reciever_node::PartPro const> & msg){
	std::stringstream ss;
	ss << msg->name << "{";

	for(auto par : msg->parameters){
		ss << " " << par.name << ": " << par.value;
	}

	ss << "}";

    ROS_INFO("I recieved part: [%s]", ss.str().c_str());
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "parts_reciever_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("part_matched", 1000, chatterCallback);

	ros::spin();

	return 0;
}
