#include <roscpp_benchmark/NodeA.h>
#include <jsoncpp/json/writer.h>

namespace roscpp_benchmark {
	NodeA::NodeA() : numberOfMeasurementsDone(0) {
		
	}
	void NodeA::calculateResults() {
		// calculate average latency
		long totalLatency = 0;
		for(uint i = 0; i < latencies.size(); i++) {
			totalLatency += latencies[i].toNSec();
		}
		long averagelatency = totalLatency / numberOfMeasurements;
		std::cout << "Average latency (nanoseconds): " << averagelatency << std::endl;
		
		// calculate standard deviation
		long totalStandardDeviation = 0;
		for(uint i = 0; i < latencies.size(); i++) {
			totalStandardDeviation += std::pow(latencies[i].toNSec() - averagelatency, 2);
		}
		std::cout << "Standard deviation: " << (std::sqrt(totalStandardDeviation / numberOfMeasurements)) << std::endl;
		
		// calculate average deviation
		long totalAverageDeviation = 0;
		for(uint i = 0; i < latencies.size(); i++) {
			totalAverageDeviation += std::abs(latencies[i].toNSec() - averagelatency);
		}
		std::cout << "Average deviation: " << (std::sqrt(totalAverageDeviation / numberOfMeasurements)) << std::endl;
	}

	void NodeA::handleMessage(const std_msgs::Int32ConstPtr& input) {
		ros::WallTime measurementEnd = ros::WallTime::now();
		latencies.push_back(measurementEnd - measurementStart);
		
		numberOfMeasurementsDone++;
		if(numberOfMeasurementsDone < numberOfMeasurements) {
			message.data = numberOfMeasurementsDone;
			measurementStart = ros::WallTime::now();
			publisher.publish(message);
		} else {
			calculateResults();
			ros::shutdown();
		}
	}
	void NodeA::onMessage(Blackboard::BlackboardSubscription & subscription, const Blackboard::OplogEntry & oplogEntry) {
		ros::WallTime measurementEnd = ros::WallTime::now();
		latencies.push_back(measurementEnd - measurementStart);
		
		numberOfMeasurementsDone++;
		if(numberOfMeasurementsDone < numberOfMeasurements) {
			blackboardMessage["data"] = numberOfMeasurementsDone;
			Json::StyledWriter writer;
			std::string output = writer.write(blackboardMessage);
			measurementStart = ros::WallTime::now();
			blackboardClient->insertDocument(output);
		} else {
			calculateResults();
			ros::shutdown();
		}
	}
	void NodeA::main() {
		std::cout << "Method: " << std::endl;
		std::cout << "1: latency topics" << std::endl;
		std::cout << "2: latency services" << std::endl;
		std::cout << "3: latency actionServers" << std::endl;
		std::cout << "4: latency blackboard" << std::endl;
		std::cout << "Choose: ";
		int method;
		std::cin >> method;
		
		std::cout << "Number of measurements: ";
		std::cin >> numberOfMeasurements;
		
		ros::NodeHandle nh;
		
		
		
		if(method == 1) {
			// topics
			publisher = nh.advertise<std_msgs::Int32>("roscpp_benchmark_a", 1);
			subscriber = nh.subscribe("roscpp_benchmark_b", 1, &NodeA::handleMessage, this);
			
			ros::Duration(5).sleep();
			
			message.data = numberOfMeasurementsDone;
			measurementStart = ros::WallTime::now();
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
		} else if(method == 4) {
			blackboardClient       = new Blackboard::BlackboardCppClient("127.0.0.1", "benchmark", "benchmark");
			blackboardSubscription = new Blackboard::BasicOperationSubscription(Blackboard::INSERT, *this);
			blackboardClient->subscribe(*blackboardSubscription);
			
			ros::Duration(5).sleep();
			
			blackboardMessage["data"] = numberOfMeasurementsDone;
			Json::StyledWriter writer;
			std::string output = writer.write(blackboardMessage);
			measurementStart   = ros::WallTime::now();
			blackboardClient->insertDocument(output);
		} else {
			std::cerr << "Unknown method" << std::endl;
			ros::shutdown();
		}
		ros::spin();
		}
	}

	int main(int argc, char **argv){
		ros::init(argc, argv, "node_a");
		
		roscpp_benchmark::NodeA node;
		node.main();
		
		return 0;
	}
