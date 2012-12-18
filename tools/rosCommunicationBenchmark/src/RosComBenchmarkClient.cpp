/**
 * @file RosComBenchmarkClient.cpp
 * @brief Communication test client.
 * @date Created: 2012-10-05
 *
 * @author Arjen van Zanten
 * @author Dennis Koole
 *
 * @section LICENSE
 * License: newBSD
 * 
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include "ros/ros.h"
#include "rosCommunicationBenchmark/TestServiceEmpty.h"
#include "rosCommunicationBenchmark/TestServiceFilled.h"
#include "std_msgs/String.h"
#include <Utilities/Utilities.h>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME_BASE "RosComBenchmarkClient_"
// @endcond

namespace rosComBenchmarkClient {
	ros::Publisher controlTopicPub;
	std::string clientID;
	int messageCount;
	ros::ServiceClient testFilledClient;
	rosCommunicationBenchmark::TestServiceFilled benchmarkService;
	std::vector<uint64_t>* sendTimes;
	std::vector<uint64_t>* returnedTimes;
	std::ofstream outputFile;
	std::string outputPathBase;

	void publishToControlTopic(std::string input) {
		std_msgs::String msg;

		std::stringstream ss;
		ss << input;
		msg.data = ss.str();

		controlTopicPub.publish(msg);
	}

	void executeBenchmark(){
		benchmarkService.request.id.client = clientID;
		rosCommunicationBenchmark::Payload payloadMsg;
		
		for(int i = 0; i < 10; i++){
			payloadMsg.testFloat = 5.0;
			benchmarkService.request.payload.push_back(payloadMsg);
		}

		for(int i = 0; i < messageCount; i++){
			benchmarkService.request.id.messageNr = i;
			(*sendTimes)[i] = ros::Time::now().toNSec();
			testFilledClient.call(benchmarkService);
			(*returnedTimes)[i] = ros::Time::now().toNSec();
		}
	}

	void storeResults(){
		outputFile.open((outputPathBase + clientID + "_client.log").c_str());
		std::cout << "Storing results" << std::endl;
		for(uint i = 0; i < sendTimes->size(); i++){
			outputFile << i << ":" << (*sendTimes)[i] << ":" << (*returnedTimes)[i] << std::endl;
		}
		outputFile.close();
	}

	void controlCallback(const std_msgs::String::ConstPtr& msg)
	{
	  	std::string message = msg->data.c_str();
	  	std::string command = message.substr(0,5);
	  	std::string parameter = message.substr(6,5);
	  	
	  	if(message.compare(0,5,"START") == 0) {
	  		std::cout << "Start command found!" << std::endl;
	  		int countdown;
	  		if(Utilities::stringToInt(countdown, message.substr(6,5).c_str()) != 0){
	  			std::cerr << "Error reading countdown time" << std::endl;
	  			return;
	  		}
	  		sleep(countdown);
	  		executeBenchmark();
	  		std::cout << "Benchmark finished" << std::endl;
	  	} else if(message.compare(0,5,"HELLO") == 0){
	  		std::cout << "Reporting to control topic." << std::endl;
	  		std::stringstream ss;
	  		ss << "CLINT:" << clientID << ":" << messageCount;
	  		publishToControlTopic(ss.str());
	  	} else if(message.compare(0,5,"STORE") == 0){
            storeResults();
        }
	}
}

using namespace rosComBenchmarkClient;
int main(int argc, char **argv)
{
	if(argc != 3 || Utilities::stringToInt(messageCount,argv[2]) != 0){
		std::cerr << "Bad arguments!" << std::endl;
		return 1;
	}
	
	clientID = argv[1];

	if(clientID.size() != 5){
		std::cerr << "Bad clientID size!" << std::endl;
		return 1;
	}

	std::stringstream nodeNameStream;
	nodeNameStream << NODE_NAME_BASE << clientID;
	ros::init(argc, argv, nodeNameStream.str());
	ros::NodeHandle nodeHandle;

	ros::Subscriber controlTopicSub = nodeHandle.subscribe("controlTopic", 1000, controlCallback);
	controlTopicPub = nodeHandle.advertise<std_msgs::String>("controlTopic", 1000);

	testFilledClient = nodeHandle.serviceClient<rosCommunicationBenchmark::TestServiceFilled>("testServiceFilled");

	sendTimes = new std::vector<uint64_t>(messageCount,0);
	returnedTimes = new std::vector<uint64_t>(messageCount,0);

	outputPathBase = "/home/arjen/benchmark/";

	ros::spin();

	return 0;
}