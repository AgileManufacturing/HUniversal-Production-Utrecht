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

#include <environmentCache/EnvironmentCache.h>
#include <iostream>

/**
 * The constructor of the EnvironmentCache class
 **/
EnvironmentCache::EnvironmentCache(): cache() {
	// Initialise services
	ros::NodeHandle nh;
	updateEnvironmentCacheService = nh.advertiseService("updateEnvironmentCache", &EnvironmentCache::updateEnvironmentCache, this);
	std::cout << "Constructor called" << std::endl;
}

/**
 * The destructor for the EnvironmentCache class
 **/
EnvironmentCache::~EnvironmentCache() {

}

/**
 * The Service that updates the environment cache
 **/
bool EnvironmentCache::updateEnvironmentCache(environmentCache::UpdateEnvironmentCache::Request &req, environmentCache::UpdateEnvironmentCache::Response &res) {
	int32_t event = req.cacheUpdate.event;
	std::string id = req.cacheUpdate.id;

	// Check which event has occured and execute correct action
	switch(event) {
		case 0: // Item is added to the environment
			if(cache.count(id) == 0) {
				
				// Insert all properties into a map
				std::map<std::string, std::string> options;
				createMapFromVector(req.cacheUpdate.properties.map, options);

				cache.insert(std::pair<std::string, std::map<std::string, std::string> >(id, options));
				std::cout << "New item added to environment cache" << std::endl;
			} else {
				std::cerr << "Item already in cache" << std::endl;
			}
			break;
		case 1: // Item is updated in workspace
			if(cache.count(req.cacheUpdate.id) == 1) {
				std::map< std::string, std::map<std::string, std::string> >::iterator cacheIt;
				cacheIt = cache.find(id);
				
				// Insert properties from message into a map
				std::map<std::string, std::string> options;
				for(int i = 0; i < (int)req.cacheUpdate.properties.map.size(); i++) {
					options.insert(std::pair<std::string, std::string>(req.cacheUpdate.properties.map[i].key, req.cacheUpdate.properties.map[i].value));
				}

				// Check if the option already exists. When it does update it, else add it
				std::map<std::string, std::string>::iterator optionsIt;
				std::map<std::string, std::string>::iterator propertyIt;
				for(optionsIt = options.begin(); optionsIt != options.end(); optionsIt++) {
					if((*cacheIt).second.count((*optionsIt).first) == 1) {
						propertyIt = (*cacheIt).second.find((*optionsIt).first);
						(*propertyIt).second = (*optionsIt).second;
					} else {
						(*cacheIt).second.insert(std::pair<std::string, std::string>((*optionsIt).first, (*optionsIt).second ));
						std::cout << "Property with key " << (*optionsIt).first << " added to cache" << std::endl;
					}
				}


				std::cout << "Item with id " << req.cacheUpdate.id << " updated" << std::endl;
			} else {
				std::cerr << "Cannot update " << std::endl;
			}
			break;
		case 2:
			if(cache.count(id) == 1) {
				cache.erase(id);
				std::cout << "Item deleted from environment cache" << std::endl;
			} else {
				std::cerr << "Item with id " << id << " cannot be deleted because it is not found in the cache" << std::endl;
			}
			break; // Item is removed from the environment
		default:
			break;
	}
	printEnvironmentCache(); // debug
	return true;
}

void EnvironmentCache::printEnvironmentCache(){
	std::map< std::string, std::map<std::string, std::string> >::iterator cacheIt;
	std::map<std::string, std::string>::iterator propertiesIt;
	for(cacheIt = cache.begin(); cacheIt != cache.end(); cacheIt++) {
		std::cout << (*cacheIt).first << " :" << std::endl;
		for(propertiesIt = (*cacheIt).second.begin(); propertiesIt != (*cacheIt).second.end(); propertiesIt++) {
			std::cout << "\t" << (*propertiesIt).first << " : " << (*propertiesIt).second << std::endl;
		}
	}
}

bool EnvironmentCache::addItemToCache() {
	return true;
}

bool EnvironmentCache::updateItemInCache() {
	return true;
}

bool EnvironmentCache::removeItemFromCache() {
	return true;
}

void EnvironmentCache::createMapFromVector(const std::vector<environmentCommunicationMessages::KeyValuePair> &properties, std::map<std::string, std::string> &optionsMap) {
	for(int i = 0; i < (int)properties.size(); i++) {
		optionsMap.insert(std::pair<std::string, std::string>(properties[i].key, properties[i].value));
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "EnvironmentCache");
	EnvironmentCache envCache;
	ros::spin();
	return 0;	
}