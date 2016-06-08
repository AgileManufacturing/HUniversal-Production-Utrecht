#include "vision_node/FoundPublisher.h"

#include "std_msgs/String.h"
#include <sstream>
#include "parts_reciever_node/PartPro.h"
#include "parts_reciever_node/PartParameter.h"

FoundPublisher::FoundPublisher(){
	ros::NodeHandle n;
	part_publisher = n.advertise<parts_reciever_node::PartPro>("part_matched", 1000);
}

void FoundPublisher::publish(Part & part){
	parts_reciever_node::PartPro msg;
	msg.name = part.name;

	for(auto parameter : part.parameters){
		parts_reciever_node::PartParameter msg_param;
		msg_param.name = parameter.first;
		msg_param.value = parameter.second;
		msg.parameters.push_back(msg_param);
	}

	part_publisher.publish(msg);
}
