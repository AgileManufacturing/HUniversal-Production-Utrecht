#include "vision_node/FoundPublisher.h"

#include "std_msgs/String.h"
#include <sstream>
#include "vision_node/PartPro.h"
#include "vision_node/PartParameter.h"

FoundPublisher::FoundPublisher(){
	ros::NodeHandle n;
	part_publisher = n.advertise<vision_node::PartPro>("part_matched", 1000);
}

void FoundPublisher::publish(Part & part){
	vision_node::PartPro msg;
	msg.name = part.name;

	for(auto parameter : part.parameters){
		vision_node::PartParameter msg_param;
		msg_param.name = parameter.first;
		msg_param.value = parameter.second;
		msg.parameters.push_back(msg_param);
	}

	part_publisher.publish(msg);
}
