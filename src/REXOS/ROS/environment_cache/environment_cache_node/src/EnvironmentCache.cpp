/**
 * @file EnvironmentCache.cpp
 * @brief The EnvironmentCache definition
 * @date Created: 2014-10-02
 *
 * @author Tommas Bakker
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

#include <environment_cache/EnvironmentCache.h>
#include <iostream>
#include <jsoncpp/json/writer.h>
#include <jsoncpp/json/reader.h>
#include <boost/algorithm/string.hpp>

#define ENVIRONMENT_CACHE_DEBUG

/**
 * The constructor of the EnvironmentCache class
 **/
EnvironmentCache::EnvironmentCache(std::string equipletName) :
		equipletName(equipletName) {
	// Initialise services
	getDataServiceServer = nh.advertiseService("getData", &EnvironmentCache::getData, this);
	setDataServiceServer = nh.advertiseService("setData", &EnvironmentCache::setData, this);
	removeDataServiceServer = nh.advertiseService("removeData", &EnvironmentCache::removeData, this);
	REXOS_INFO("environment cache started");
}

bool EnvironmentCache::getData(environment_cache::getData::Request& req, environment_cache::getData::Response& res) {
	ROS_INFO("Getting data");
	Json::StyledWriter writer;
	try {
		res.jsonData = writer.write(cache.getItemDataInCache(req.identifier, req.paths));
		ROS_INFO_STREAM(res.jsonData);
		return true;
	} catch (std::runtime_error ex) {
		return false;
	}
}
bool EnvironmentCache::setData(environment_cache::setData::Request& req, environment_cache::setData::Response& res) {
	ROS_INFO("Setting data");
	Json::Reader reader;
	Json::Value jsonNode;
	if(reader.parse(req.json, jsonNode) == false) return false;
	ROS_INFO_STREAM(jsonNode);
	cache.setItemDataInCache(req.identifier, jsonNode);
	return true;
}
bool EnvironmentCache::removeData(environment_cache::removeData::Request& req, environment_cache::removeData::Response& res) {
	ROS_INFO("Removing data");
	cache.removeDataOfItemFromCache(req.identifier, req.paths);
	return true;
}

/** 
 * Main that creates the environment cache
 **/
int main(int argc, char **argv){
	if(argc < 2){
		REXOS_ERROR("Usage: environment_cache (--isShadow) equipletName");
		return -1;
	}
	
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--isShadow") {
			isShadow = true;
		}
	}
	
	std::string equipletName = std::string(argv[argc - 1]);
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_environmentCache";
	ros::init(argc, argv, nodeName);
	
	EnvironmentCache environmentCache(equipletName);
	
	ros::spin();
	return 0;
}
