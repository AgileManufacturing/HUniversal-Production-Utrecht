/**
 * @file lookup_handler.h
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

#ifndef LOOKUPHANDLER_H
#define LOOKUPHANDLER_H

#include "ros/ros.h"
#include <environment_communication_msgs/KeyValuePair.h>
#include <environment_communication_msgs/Map.h>
#include <environment_cache/LookupEnvironmentObject.h>
#include "lookup_handler/LookupServer.h"
#include "lookup_handler/LookupMessage.h"

namespace EnvironmentCommunication{
	/** 
	 * Will request data from environmentcache, and combine the data with the data in the lookupRequest. After combining it, send it back to equipletNode
	 **/
	class LookupHandler{
		public:
			LookupHandler();
		private:
			bool lookupServiceCallback(lookup_handler::LookupServer::Request &req, lookup_handler::LookupServer::Response &rep);
			/**
			 * @var ros::ServiceClient lookupClient
			 * the client that will call the service in environmentCache
			 **/
			ros::ServiceClient lookupClient;
			/**
			 * @var ros::ServiceServer lookupServer
			 * the service for the equipletNode
			 **/
			ros::ServiceServer lookupServer;

			void createMapFromVector(const std::vector<environment_communication_msgs::KeyValuePair> &vector, std::map<std::string, std::string> &map);
	};
}
#endif
