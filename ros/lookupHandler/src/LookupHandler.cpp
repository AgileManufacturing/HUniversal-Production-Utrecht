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

#include "lookupHandler/LookupHandler.h"

/**
 * Create a LookupHandler
 **/
EnvironmentCommunication::LookupHandler::LookupHandler(){
	ros::NodeHandle nodeHandle;
	lookupClient = nodeHandle.serviceClient<environmentCache::LookupEnvironmentObject>("LookupEnvironmentObject");
	lookupServer = nodeHandle.advertiseService("LookupHandler/lookup", &LookupHandler::lookupServiceCallback, this);
	ROS_INFO("LookupHandler constructor called");
}

/**
 * Call back for lookupHandler/lookup service
 * Will lookup data in environmentcache and use the payload of request on the data
 * @param request Contains the data for the lookup in the cache
 * @param response Will contain the data from the environment cache, if it was found
 * @return returns always true
 **/
bool EnvironmentCommunication::LookupHandler::lookupServiceCallback(lookupHandler::LookupServer::Request &request, lookupHandler::LookupServer::Response &response){
	// Construct a message for LookupEnvironmentObject service
	environmentCache::LookupEnvironmentObject msg;	
	msg.request.lookupID = request.lookupMsg.lookupID;
	std::string payloadRequest = request.lookupMsg.payLoad;

	if(lookupClient.call(msg)) {
		if(msg.response.found) {
			JSONNode payload = libjson::parse(payloadRequest);
			JSONNode envCache = libjson::parse(msg.response.properties);

			// First rotate the position inside the crate around the middlepoint (0, 0)
			JSONNode rotatedPoint = rotate(payload, envCache);
			// Then translate the point to the workspace coordinate system by adding it to the envCache pos
			JSONNode translatedPoint = translateObjects(rotatedPoint, envCache);

			response.succeeded = true;
		} else {
			response.succeeded = false;
		}
	} 
	return true;
}

/** 
 * Will rotate the point in the payload with the angle saved in the environmentCache
 * @param payload Contains the data of a point in the object space
 * @param envCache Contains the data saved about the object in the environmentcache
 * @return returns always true
 **/
JSONNode EnvironmentCommunication::LookupHandler::rotate(JSONNode payload, JSONNode envCache) {
	float angle = payload["angle"].as_float();
	float radians = (angle * M_PI) / 180.0;
	float x = payload["x"].as_float();
	float y = payload["y"].as_float();
	float newX = (x * cos(radians)) - (y * sin(radians));
	float newY = (x * sin(radians)) + (y * cos(radians));

	JSONNode result(JSON_NODE);
	result.push_back(JSONNode("x", newX));
	result.push_back(JSONNode("y", newY));
	return result;
}

/** 
 * Will translate the point in the payload with the point saved in the environmentCache
 * @param payload Contains the data of a point in the object space
 * @param envCache Contains the data saved about the object in the environmentcache
 * @return returns always true
 **/
JSONNode EnvironmentCommunication::LookupHandler::translateObjects(JSONNode payload, JSONNode envCache) {
	JSONNode::const_iterator payloadInterator = payload.begin();
	JSONNode result(JSON_NODE);
	while(payloadInterator != payload.end()) {
		std::string node_name = payloadInterator -> name();
		 // recursively call ourselves to dig deeper into the tree
        if (payloadInterator -> type() == JSON_ARRAY || payloadInterator -> type() == JSON_NODE){
            translateObjects(*payloadInterator, envCache[node_name]);
        }

        bool envResult = false;
		if(envCache.find(node_name) != envCache.end() ) {
			envResult = true;
		}

    	int payloadVar = payloadInterator -> as_float();
    	if(envResult) { 
        	int envCacheVar = envCache[node_name].as_float();
        	int translationResult = payloadVar + envCacheVar;
        	result.push_back(JSONNode(node_name, translationResult));
    	} 
		++payloadInterator;
	}
	return result;
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
