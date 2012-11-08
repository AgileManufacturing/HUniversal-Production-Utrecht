/**
 * @file EnvironmentCache.cpp
 * @brief The EnvironmentCache implementation
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

#include "ros/ros.h"
#include <environmentCache/EnvironmentCache.h>

/**
 * The constructor of the EnvironmentCache class
 **/
EnvironmentCache::EnvironmentCache(): cache() {
	// Initialise services
	ros::NodeHandle nh;
	updateEnvironmentCacheService = nh.advertiseService("UpdateEnvironmentCache", &EnvironmentCache::updateEnvironmentCache, this);
}

/**
 * The destructor for the EnvironmentCache class
 **/
EnvironmentCache::~EnvironmentCache() {

}

/**
 * The Service that updates the environment cache
 **/
bool EnvironmentCache::updateEnvironmentCache(environmentCache::UpdateEnvironmentCache &req, environmentCache::UpdateEnvironmentCache &res) {
	int32_t event = req.event;
	// Check which event has occured and execute correct action
	switch(event) {
		case 0: // Item is added to the environment
			if(cache.count(req.id) == 0) {
				// Insert all properties into a map
				std::map<std::string, std::string> properties;
				for(int i = 0; i < req.properties.size(); i++) {
					cache.insert(std::pair<std::string, std::string>(properties[i].key, properties[i].value));
				}
				cache.insert(req.id, properties);
				std::cout << "New item added to environment cache" << std::endl;
			} else {
				std::cerr << "Item already in cache" << std::endl;
			}
			break;
		case 1: // Item is updated in workspace
			break;
		case 2: 
			break; // Item is removed from the environment
		default:
			break;
	}
}

int main(int argc, char **argv) {
	EnvironmentCache envCache();
	return 0;	
}