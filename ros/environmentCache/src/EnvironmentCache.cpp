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
EnvironmentCache::EnvironmentCache() : cache(), nodeHandle(){
	// Initialise services
	lookupEnvironmentObjectService = nodeHandle.advertiseService("LookupEnvironmentObject", &EnvironmentCache::lookupEnvironmentObject, this);
	updateEnvironmentCacheService = nodeHandle.advertiseService("updateEnvironmentCache", &EnvironmentCache::updateEnvironmentCache, this);
}

/**
 * Get the properties of an item in the cache
 *
 * @param req The request object. Contains the id of the item that is in the cache
 * @param res The response object. Contains a boolean that is true when item was found, else false.
 * Also contains the Map object with the properties of the item. Empty when there is not an item found with the id
 *
 * @return returns true
 **/
bool EnvironmentCache::lookupEnvironmentObject(environmentCache::LookupEnvironmentObject::Request &req, environmentCache::LookupEnvironmentObject::Response &res){
	/*std::string id = req.lookupID;
	if(cache.count(id)){
		std::map< std::string, std::map<std::string, std::string> >::iterator cacheIterator;
		cacheIterator = cache.find(id);
		std::map<std::string, std::string> properties = (*cacheIterator).second;
		res.object = createMapMessageFromProperties(properties);
		res.found = true;
	} else{
		rexosStdMsgs::Map map;
		res.object = map;
		res.found = false;
	}*/
	return true;
}

/**
 * The Service that updates the environment cache
 *
 * @param req The request for this service with an EnvironmentCacheUpdate message
 * @param res The reponse, contains a boolean success. True if success else false
 *
 * @return true
 **/
bool EnvironmentCache::updateEnvironmentCache(environmentCache::UpdateEnvironmentCache::Request &req, environmentCache::UpdateEnvironmentCache::Response &res){
	int32_t event = req.cacheUpdate.event;
	const std::string &id = req.cacheUpdate.id;
	bool success = false;
	// Check which event has occured and perform correct action
	switch(event){
		case ADD:
			// Item is added to the cache
			if(addItemToCache(id, req.cacheUpdate.properties)){
				success = true;
			}
			break;
		case UPDATE:
			// Item is updated in cache
			if(updateItemInCache(id, req.cacheUpdate.properties)){
				success = true;
			}
		case REMOVE:
			// Remove item from cache
			if(removeItemFromCache(id)){
				success = true;
			}
			break;
		default:
			break;
	}
	// Print the cache, for debugging properties
	res.success = success;
	//printEnvironmentCache();
	return true;
}

/**
 * Print all the items in the cache to standard output
 **/
void EnvironmentCache::printEnvironmentCache(){
	std::map< std::string, std::string >::iterator cacheIterator;
	for(cacheIterator = cache.begin(); cacheIterator != cache.end(); cacheIterator++){
		ROS_INFO("%s : %s", cacheIterator->first.c_str(), cacheIterator->second.c_str());
	}
}

/**
 * Add a new item to the cache
 *
 * @param id The id of the item to add
 * @param properties The json valid properties string
 *
 * @return True if item is added, false if there is already an item in the cache with the same id of the item to add
 **/
bool EnvironmentCache::addItemToCache(const std::string &id, const std::string &properties){
	if(cache.count(id) == 0){
		cache.insert(std::pair<std::string, std::string >(id, properties));
		ROS_INFO("Item with id %s added to environment cache: %s", id.c_str(), properties.c_str());
		return true;
	}
	return false;
}

/**
 * Update an item in the cache.
 *
 * @param id The id of the item to update
 * @param properties The vector with properties to update. If a property in this vector already exists in property list of the item it 
 * will be updated, else it will be added to the list of properties
 *
 * @return True if an item with the id is found in the cache, else false
 **/
bool EnvironmentCache::updateItemInCache(const std::string &id, const std::string &properties){
	std::map<std::string, std::string>::iterator it = cache.find(id);
	if(it != cache.end()) {
		it->second = properties;
		ROS_INFO("Properties of item with id %s updated: %s", it->first.c_str(), it->second.c_str());
		return true;
	}
	return false;
}

/**
 * Remove an item from the cache
 *
 * @param id The id of the item to delete
 *
 * @return True if item is deleted, false if no item with the id specified is found
 **/
bool EnvironmentCache::removeItemFromCache(const std::string &id){
	if(cache.count(id) == 1){
		cache.erase(id);
		ROS_INFO("Item with id %s removed from cache", id.c_str());
		return true;
	}
	return false;
}

/**
 * Main that creates the environment cache
 **/
int main(int argc, char **argv){
	ros::init(argc, argv, "EnvironmentCache");
	EnvironmentCache envCache;
	ros::spin();
	return 0;
}