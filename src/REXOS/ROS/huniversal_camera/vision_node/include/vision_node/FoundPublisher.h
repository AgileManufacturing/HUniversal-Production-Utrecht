
#pragma once

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"
#include "vision_node/Part.h"

class FoundPublisher{
public:
	FoundPublisher();
	void publish(Part & part);
private:
	ros::Publisher part_publisher;
};
