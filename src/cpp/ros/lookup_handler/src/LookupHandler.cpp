/**
 * @file LookupHandler.cpp
 * @brief Names for the DeltaRobot services.
 * @date Created: 2012-11-07
 *
 * @author Arjan Groenewegen
 *
 * @section LICENSE
 * License: newBSD
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

#include "lookup_handler/LookupHandler.h"

/**
 * Create a LookupHandler
 **/
EnvironmentCommunication::LookupHandler::LookupHandler(){
	ros::NodeHandle nodeHandle;
	lookupClient = nodeHandle.serviceClient<environment_cache::LookupEnvironmentObject>("LookupEnvironmentObject");
	lookupServer = nodeHandle.advertiseService("LookupHandler/lookup", &LookupHandler::lookupServiceCallback, this);
	ROS_INFO("LookupHandler constructor called");
}

/**
 * Call back for lookupHandler/lookup service
 * Will lookup data in environmentcache and use the payload of request on the data
 * @param request Contains the data for the lookup in the cache
 * @param response Will contain the data from the cache, if it was found
 **/
bool EnvironmentCommunication::LookupHandler::lookupServiceCallback(lookup_handler::LookupServer::Request &request, lookup_handler::LookupServer::Response &response){
	// Construct a message for LookupEnvironmentObject service
	environment_cache::LookupEnvironmentObject msg;
	msg.request.lookupID = request.lookupMsg.lookupID;
	std::map<std::string, std::string> payLoadMap;
	std::map<std::string, std::string>::iterator it;
	createMapFromVector(request.lookupMsg.payLoad.map, payLoadMap);
	if(lookupClient.call(msg)){
		if(msg.response.found){
			for(int i = 0; i < (int)(msg.response.object.map.size()); i++){
				it = payLoadMap.find(msg.response.object.map[i].key);
				std::string valueObject =  (*it).second;
				std::string valuePayload = msg.response.object.map[i].value;

				// TODO should do something cool with the data

				std::cout << "valueObject " << valueObject << " valuePayload " << valuePayload << std::endl;
			}
			response.succeeded = true;
		} else
			response.succeeded = false;
	} else{
		response.succeeded = false;
		ROS_ERROR("Error in calling LookupEnvironmentObject");
	}
	return true;
}

/**
 * Convert a vector to a map
 *
 * @param vector The vector with KeyValuePair objects
 * @param map The map where the keys and values of the objects in the vector are inserted to
 **/
void EnvironmentCommunication::LookupHandler::createMapFromVector(const std::vector<environment_communication_msgs::KeyValuePair> &vector, std::map<std::string, std::string> &map){
	for(int i = 0; i < (int)vector.size(); i++){
		map.insert(std::pair<std::string, std::string>(vector[i].key, vector[i].value));
	}
}

/**
 * Main function for the lookupHandler
 **/
int main(int argc, char **argv){
	ros::init(argc, argv, "lookupHandlerNode");
	EnvironmentCommunication::LookupHandler lh;
	ros::spin();
	return 0;
}
