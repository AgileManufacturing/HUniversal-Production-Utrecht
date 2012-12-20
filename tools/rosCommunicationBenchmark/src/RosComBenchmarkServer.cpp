/**
 * @file RosComBenchmarkServer.cpp
 * @brief Communication test server.
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
#include <cstdio>
#include <iostream>
#include <map>
#include <vector>
#include <Utilities/Utilities.h>
#include <fstream>

// @cond HIDE_NODE_NAME_FROM_DOXYGEN
#define NODE_NAME "RosComBenchmarkServer"
// @endcond

namespace rosComBenchmarkServer {
    std::map<std::string, std::vector<uint64_t> > results;
    ros::Publisher controlTopicPub;
    std::ofstream outputFile;
    std::string outputPathBase;

    void publishToControlTopic(std::string input) {
    	std_msgs::String msg;

    	std::stringstream ss;
    	ss << input;
    	msg.data = ss.str();

    	controlTopicPub.publish(msg);
    }

    void storeResults() {
        std::map<std::string, std::vector<uint64_t> >::iterator it;
        for(it = results.begin(); it != results.end(); it++){
            std::cout << "Storing " << (*it).first << " results" << std::endl;
            outputFile.open((outputPathBase + (*it).first + "_server.log").c_str());
            
            for(uint i = 0; i < (*it).second.size(); i++){
                outputFile << i << ":" << (*it).second[i] << std::endl;
            }

            outputFile.close();
        }
    }

    bool testServiceEmpty(rosCommunicationBenchmark::TestServiceEmpty::Request &req, rosCommunicationBenchmark::TestServiceEmpty::Response &res) {
    	// rcvTimes.push_back(ros::Time::now().toNSec());

    	return true;
    }

    bool testServiceFilled(rosCommunicationBenchmark::TestServiceFilled::Request &req, rosCommunicationBenchmark::TestServiceFilled::Response &res) {
    	// results[req.id.client][req.id.messageNr] = ros::Time::now().toNSec();

        res.returnPayload.testString = req.payload.testString; 

        return true;
    }

    void controlCallback(const std_msgs::String::ConstPtr& msg)
    {	
        std::string message = msg->data.c_str();
      	std::string command = message.substr(0,5);
      	std::string parameter = message.substr(6,5);
      	
      	if(message.compare(0,5,"HELLO") == 0){
      		std::cout << "Reporting to control topic." << std::endl;
      		// server gets the ID 0 for now
      		publishToControlTopic("SERVR:0");
      	} else if(message.compare(0,5,"CLINT") == 0){
      		std::string clientName = message.substr(6,5);
      		int clientMessageCount; 
      		if(Utilities::stringToInt(clientMessageCount, message.substr(12,100).c_str()) != 0){
      			std::cerr << "Error converting string to int!" << std::endl;
      			publishToControlTopic("ERROR:KILLD");
      			exit(1);
      		}	
      		
      		std::cout << "Adding new client " << clientName << " to results table with " << clientMessageCount << " places" << std::endl;
			std::vector<uint64_t>* clientResults = new std::vector<uint64_t>(clientMessageCount,0);			
			results.insert(std::make_pair(clientName,*clientResults));
      	} else if(message.compare(0,5,"STORE") == 0){
            storeResults();
		std::cout << "stored" << std::endl;
        }
    }
}

using namespace rosComBenchmarkServer;

int main(int argc, char **argv){
	ros::init(argc, argv, NODE_NAME);

	ros::NodeHandle n;
	ros::ServiceServer emptyService = n.advertiseService("testServiceEmpty", testServiceEmpty);
	ros::ServiceServer filledService = n.advertiseService("testServiceFilled", testServiceFilled);

	ros::Subscriber controlTopicSub = n.subscribe("controlTopic", 1000, controlCallback);

	controlTopicPub = n.advertise<std_msgs::String>("controlTopic", 1000);

    outputPathBase = "/home/agileman/benchmark/";

	ros::spin();
	return 0;
}
