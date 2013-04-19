/**
 * @file EnvironmentCache.h
 * @brief The EnvironmentCache definition
 * @date Created: 2012-11-07
 *
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

#ifndef ENVIRONMENTCACHE_H
#define ENVIRONMENTCACHE_H

#include "ros/ros.h"
#include <environment_cache/UpdateEnvironmentCache.h>
#include <environment_cache/LookupEnvironmentObject.h>
#include <string>
#include <map>

/**
 * This class represents the environment cache
 **/
class EnvironmentCache{
public:
	/**
	 * @var environmentAction
	 * The enum for environment actions that are possible
	 **/
	enum environmentAction {ADD, UPDATE, REMOVE};
	EnvironmentCache();
	bool lookupEnvironmentObject(environment_cache::LookupEnvironmentObject::Request &req, environment_cache::LookupEnvironmentObject::Response &res);
	bool updateEnvironmentCache(environment_cache::UpdateEnvironmentCache::Request &req, environment_cache::UpdateEnvironmentCache::Response &res);
	void printEnvironmentCache();

private:
	bool addItemToCache(std::string id, const std::vector<environment_communication_msgs::KeyValuePair> &properties);
	bool updateItemInCache(std::string id, const std::vector<environment_communication_msgs::KeyValuePair> &properties);
	bool removeItemFromCache(std::string id);
	void createMapFromVector(const std::vector<environment_communication_msgs::KeyValuePair> &propertiesVector, std::map<std::string, std::string> &propertiesMap);
	environment_communication_msgs::Map createMapMessageFromProperties(std::map<std::string, std::string> &properties);

	/**
	 * @var std::map< std::string, std::map<std::string, std::string> > cache
	 * Map that is used for caching the data
	 **/
	std::map< std::string, std::map<std::string, std::string> > cache;

	/**
	 * @var ros::ServiceServer lookupEnvironmentObjectService
	 * The service object for calling the lookup environment
	 **/
	ros::ServiceServer lookupEnvironmentObjectService;

	/**
	 * @var ros::ServiceServer updateEnvironmentCacheService
	 * Server thats gets called to update the data in the update environment cache
	 **/
	ros::ServiceServer updateEnvironmentCacheService;
};
#endif
