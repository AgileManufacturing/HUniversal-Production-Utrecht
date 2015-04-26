#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <roscpp_benchmark/benchmarkAction.h>
#include <iostream>
#include <vector>

ros::Publisher publisher;
ros::Subscriber subscriber;
ros::ServiceClient serviceClient;

std_msgs::Int32 message;
std_srvs::Empty call;
roscpp_benchmark::benchmarkGoal goal;

int numberOfMeasurements;
int numberOfMeasurementsDone = 0;
ros::WallTime measurementStart;
std::vector<ros::WallDuration> latencies;

void calculateResults() {
	// calculate average latency
	long totalLatency = 0;
	for(uint i = 0; i < latencies.size(); i++) {
		totalLatency += latencies[i].toNSec();
	}
	long averagelatency = totalLatency / numberOfMeasurements;
	ROS_INFO_STREAM("Average latency (nanoseconds): " << averagelatency);
	
	// calculate standard deviation
	long totalStandardDeviation = 0;
	for(uint i = 0; i < latencies.size(); i++) {
		totalStandardDeviation += std::pow(latencies[i].toNSec() - averagelatency, 2);
	}
	ROS_INFO_STREAM("Standard deviation: " << (std::sqrt(totalStandardDeviation / numberOfMeasurements)));
	
	// calculate average deviation
	long totalAverageDeviation = 0;
	for(uint i = 0; i < latencies.size(); i++) {
		totalAverageDeviation += std::abs(latencies[i].toNSec() - averagelatency);
	}
	ROS_INFO_STREAM("Average deviation: " << (std::sqrt(totalAverageDeviation / numberOfMeasurements)));
}

void handleMessage(const std_msgs::Int32ConstPtr& input) {
	ros::WallTime measurementEnd = ros::WallTime::now();
	latencies.push_back(measurementEnd - measurementStart);
	
	numberOfMeasurementsDone++;
	if(numberOfMeasurementsDone < numberOfMeasurements) {
		measurementStart = ros::WallTime::now();
		message.data = numberOfMeasurementsDone;
		publisher.publish(message);
	} else {
		calculateResults();
		ros::shutdown();
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "node_a");
	
	std::cout << "Method: " << std::endl;
	std::cout << "1: latency topics" << std::endl;
	std::cout << "2: latency services" << std::endl;
	std::cout << "3: latency actionServers" << std::endl;
	std::cout << "Choose: ";
	int method;
	std::cin >> method;
	
	std::cout << "Number of measurements: ";
	std::cin >> numberOfMeasurements;
	
	ros::NodeHandle nh;
	
	
	
	if(method == 1) {
		// topics
		publisher = nh.advertise<std_msgs::Int32>("roscpp_benchmark_a", 1);
		subscriber = nh.subscribe("roscpp_benchmark_b", 1, handleMessage);
		
		ros::Duration(5).sleep();
		
		measurementStart = ros::WallTime::now();
		message.data = numberOfMeasurementsDone;
		publisher.publish(message);
	} else if(method == 2) {
		// services
		serviceClient = nh.serviceClient<std_srvs::Empty>("roscpp_benchmark_service");
		
		for(int i = 0; i < numberOfMeasurements; i++) {
			measurementStart = ros::WallTime::now();
			serviceClient.call(call);
			
			ros::WallTime measurementEnd = ros::WallTime::now();
			latencies.push_back(measurementEnd - measurementStart);
			
			numberOfMeasurementsDone++;
			if(numberOfMeasurementsDone == numberOfMeasurements) {
				calculateResults();
				ros::shutdown();
			}
		}
	} else if(method == 3) {
		// actionServers
		actionlib::SimpleActionClient<roscpp_benchmark::benchmarkAction> actionClient("roscpp_benchmark_action", true);
		actionClient.waitForServer();
		
		for(int i = 0; i < numberOfMeasurements; i++) {
			measurementStart = ros::WallTime::now();
			actionClient.sendGoal(goal);
			actionClient.waitForResult();
			
			ros::WallTime measurementEnd = ros::WallTime::now();
			latencies.push_back(measurementEnd - measurementStart);
			
			numberOfMeasurementsDone++;
			if(numberOfMeasurementsDone == numberOfMeasurements) {
				calculateResults();
				ros::shutdown();
			}
		}
	}
	ros::spin();
	
	return 0;
}
