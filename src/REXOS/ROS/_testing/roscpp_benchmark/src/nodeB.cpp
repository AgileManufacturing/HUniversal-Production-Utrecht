#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <roscpp_benchmark/benchmarkAction.h>
#include <iostream>
#include <vector>

ros::Publisher publisher;
ros::Subscriber subscriber;
ros::ServiceServer serviceServer;
std_msgs::Int32 message;

void handleMessage(const std_msgs::Int32ConstPtr& input) {
	message.data = input->data;
	publisher.publish(message);
}

bool handleCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	return true;
}

void handleGoal(const roscpp_benchmark::benchmarkGoalConstPtr& goal, 
		actionlib::SimpleActionServer<roscpp_benchmark::benchmarkAction>* as) {
	as->setSucceeded();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "node_b");
	
	std::cout << "Method: " << std::endl;
	std::cout << "1: latency topics" << std::endl;
	std::cout << "2: latency services" << std::endl;
	std::cout << "3: latency actionServers" << std::endl;
	std::cout << "Choose: ";
	int method;
	std::cin >> method;
	
	ros::NodeHandle nh;
	
	if(method == 1) {
		// topics
		publisher = nh.advertise<std_msgs::Int32>("roscpp_benchmark_b", 1);
		subscriber = nh.subscribe("roscpp_benchmark_a", 1, handleMessage);
		ros::spin();
	} else if(method == 2) {
		serviceServer = nh.advertiseService("roscpp_benchmark_service", handleCall);
		ros::spin();
	} else if(method == 3) {
		actionlib::SimpleActionServer<roscpp_benchmark::benchmarkAction> actionServer(nh, "roscpp_benchmark_action", 
				boost::bind(&handleGoal, _1, &actionServer), false);
		actionServer.start();
		ros::spin();
	}
	
	return 0;
}
