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

#include <environment_cache/EnvironmentCache.h>
#include <iostream>

/**
 * The constructor of the EnvironmentCache class
 **/
EnvironmentCache::EnvironmentCache() : cache(){
	// Initialise services
	ros::NodeHandle nh;
	lookupEnvironmentObjectService = nh.advertiseService("LookupEnvironmentObject", &EnvironmentCache::lookupEnvironmentObject, this);
	updateEnvironmentCacheService = nh.advertiseService("updateEnvironmentCache", &EnvironmentCache::updateEnvironmentCache, this);
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
bool EnvironmentCache::lookupEnvironmentObject(environment_cache::LookupEnvironmentObject::Request &req, environment_cache::LookupEnvironmentObject::Response &res){
	std::string id = req.lookupID;
	if(cache.count(id)){
		std::map< std::string, std::map<std::string, std::string> >::iterator cacheIterator;
		cacheIterator = cache.find(id);
		std::map<std::string, std::string> properties = (*cacheIterator).second;
		res.object = createMapMessageFromProperties(properties);
		res.found = true;
	} else{
		environment_communication_msgs::Map map;
		res.object = map;
		res.found = false;
	}
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
bool EnvironmentCache::updateEnvironmentCache(environment_cache::UpdateEnvironmentCache::Request &req, environment_cache::UpdateEnvironmentCache::Response &res){
	int32_t event = req.cacheUpdate.event;
	std::string id = req.cacheUpdate.id;
	bool success = false;

	// Check which event has occured and perform correct action
	switch(event){
		case ADD:
			// Item is added to the cache
			if(addItemToCache(id, req.cacheUpdate.properties.map)){
				success = true;
			}
			break;
		case UPDATE:
			// Item is updated in cache
			if(updateItemInCache(id, req.cacheUpdate.properties.map)){
				success = true;
			}
			break;
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
	printEnvironmentCache();
	res.success = success;
	return true;
}

/**
 * Print all the items in the cache to standard output
 **/
void EnvironmentCache::printEnvironmentCache(){
	std::map< std::string, std::map<std::string, std::string> >::iterator cacheIterator;
	std::map<std::string, std::string>::iterator propertiesIterator;
	for(cacheIterator = cache.begin(); cacheIterator != cache.end(); cacheIterator++){
		std::cout << (*cacheIterator).first << " :" << std::endl;
		for(propertiesIterator = (*cacheIterator).second.begin(); propertiesIterator != (*cacheIterator).second.end(); propertiesIterator++){
			std::cout << "\t" << (*propertiesIterator).first << " : " << (*propertiesIterator).second << std::endl;
		}
	}
}

/**
 * Add a new item to the cache
 *
 * @param id The id of the item to add
 * @param properties The vector of properties
 *
 * @return True if item is added, false if there is already an item in the cache with the same id of the item to add
 **/
bool EnvironmentCache::addItemToCache(std::string id, const std::vector<environment_communication_msgs::KeyValuePair> &properties){
	if(cache.count(id) == 0){
		std::map<std::string, std::string> options;
		// Convert the vector with properties to a map
		createMapFromVector(properties, options);
		cache.insert(std::pair<std::string, std::map<std::string, std::string> >(id, options));
		std::cout << "New item added to environment cache" << std::endl;
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
bool EnvironmentCache::updateItemInCache(std::string id, const std::vector<environment_communication_msgs::KeyValuePair> &properties){
	if(cache.count(id) == 1){
		// Create iterator for the cache
		std::map< std::string, std::map<std::string, std::string> >::iterator cacheIterator;
		// Let the iterator point to the Item with the id specified in the update
		cacheIterator = cache.find(id);

		// Convert the vector of properties in the update message to a map
		std::map<std::string, std::string> propertiesMap;
		createMapFromVector(properties, propertiesMap);

		// Loop through all the properties that are specified in the update
		std::map<std::string, std::string>::iterator optionsIterator;
		std::map<std::string, std::string>::iterator propertyIterator;
		for(optionsIterator = propertiesMap.begin(); optionsIterator != propertiesMap.end(); optionsIterator++){
			// If the property is found update it, else add it to the list of properties
			if((*cacheIterator).second.count((*optionsIterator).first) == 1){
				propertyIterator = (*cacheIterator).second.find((*optionsIterator).first); // Create iterator to the property found
				(*propertyIterator).second = (*optionsIterator).second; // update its value
				std::cout << "Property with key " << (*propertyIterator).first << " in item with id " << (*cacheIterator).first << " updated" << std::endl;
			} else{
				(*cacheIterator).second.insert(std::pair<std::string, std::string>((*optionsIterator).first, (*optionsIterator).second ));
				std::cout << "Property with key " << (*optionsIterator).first << " added to item with id " << id << std::endl;
			}
		}
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
bool EnvironmentCache::removeItemFromCache(std::string id){
	if(cache.count(id) == 1){
		cache.erase(id);
		std::cout << "Item with id " << id << " deleted from cache" << std::endl;
		return true;
	}
	return false;
}

/**
 * Convert a vector to a map
 *
 * @param propertiesVector The vector with KeyValuePair objects
 * @param propertiesMap The map where the keys and values of the objects in the vector are inserted to
 **/
void EnvironmentCache::createMapFromVector(const std::vector<environment_communication_msgs::KeyValuePair> &propertiesVector, std::map<std::string, std::string> &propertiesMap){
	for(int i = 0; i < (int)propertiesVector.size(); i++){
		propertiesMap.insert(std::pair<std::string, std::string>(propertiesVector[i].key, propertiesVector[i].value));
	}
}

/**
 * Create a Map message from a map with strings as keys and strings as values
 *
 * @param properties The map to convert
 *
 * @return environment_communication_msgs::Map The map message object
 **/
environment_communication_msgs::Map EnvironmentCache::createMapMessageFromProperties(std::map<std::string, std::string> &properties){
	std::map<std::string, std::string>::iterator propertiesIterator;
	environment_communication_msgs::Map mapMsg;
	environment_communication_msgs::KeyValuePair prop;
	for(propertiesIterator = properties.begin(); propertiesIterator != properties.end(); propertiesIterator++){
		prop.key = (*propertiesIterator).first;
		prop.value = (*propertiesIterator).second;
		mapMsg.map.push_back(prop);
	}
	return mapMsg;
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