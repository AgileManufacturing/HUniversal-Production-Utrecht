#pragma once

#include <ros/ros.h>
#include <rexos_blackboard_cpp_client/BlackboardCppClient.h>
#include <rexos_blackboard_cpp_client/BlackboardSubscriber.h>
#include <rexos_blackboard_cpp_client/FieldUpdateSubscription.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <roscpp_benchmark/benchmarkAction.h>
#include <jsoncpp/json/value.h>
#include <iostream>
#include <vector>

namespace roscpp_benchmark {
	class NodeA : public Blackboard::BlackboardSubscriber {
	public:
		NodeA();
		void main();
	protected:
		void onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry);
	private:
		ros::Publisher publisher;
		ros::Subscriber subscriber;
		ros::ServiceClient serviceClient;
		Blackboard::BlackboardCppClient *blackboardClient;
		Blackboard::BlackboardSubscription* blackboardSubscription;

		std_msgs::Int32 message;
		std_srvs::Empty call;
		roscpp_benchmark::benchmarkGoal goal;
		Json::Value blackboardMessage;

		int numberOfMeasurements;
		int numberOfMeasurementsDone;
		ros::WallTime measurementStart;
		std::vector<ros::WallDuration> latencies;
		
		void calculateResults();
		void handleMessage(const std_msgs::Int32ConstPtr& input);
		
	};
}
