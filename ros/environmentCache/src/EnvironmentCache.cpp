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
	lookupEnvironmentObjectService = nh.advertiseService("LookupEnvironmentObject", &EnvironmentCache::lookupEnvironmentObject, this);
	updateEnvironmentCacheService = nh.advertiseService("updateEnvironmentCache", &EnvironmentCache::updateEnvironmentCache, this);
	std::cout << "Constructor called" << std::endl;
}

/**
 * The destructor for the EnvironmentCache class
 **/
EnvironmentCache::~EnvironmentCache() {

}

/**
 * Get the properties of an item in the cache
 *
 * @param req the request object. Contains the id of the item to get the properties from
 * @param res The response object. Contains a boolean that is true when item was found, else false.
 * Contains also the Map object with the properties of the item. Empty when there is not an item found with the id
 **/
bool EnvironmentCache::lookupEnvironmentObject(environmentCache::LookupEnvironmentObject::Request &req, environmentCache::LookupEnvironmentObject::Response &res) {
	std::string id = req.lookupID;
	if(cache.count(id)) {
		std::map< std::string, std::map<std::string, std::string> >::iterator cacheIt;
		cacheIt = cache.find(id);
		std::map<std::string, std::string> properties = (*cacheIt).second;
		res.object = createMapMessageFromProperties(properties);
		res.found = true;
	} else{
		environmentCommunicationMessages::Map map;
		res.object = map;
		res.found = false;
	}
	return true;
}

/**
 * The Service that updates the environment cache
 *
 * @param environmentCache::UpdateEnvironmentCache::Request &req The request for this service with an EnvironmentCacheUpdate message
 * @param environmentCache::UpdateEnvironmentCache::Response &res with a bool. True if success else false
 *
 * @return true if success else false
 **/
bool EnvironmentCache::updateEnvironmentCache(environmentCache::UpdateEnvironmentCache::Request &req, environmentCache::UpdateEnvironmentCache::Response &res) {
	int32_t event = req.cacheUpdate.event;
	std::string id = req.cacheUpdate.id;
	bool success = false;

	// Check which event has occured and perform correct action
	switch(event) {
		case 0: // Item is added to the cache
			if(addItemToCache(id, req.cacheUpdate.properties.map)) {
				success = true;
			}
			break;
		case 1: // Item is updated in cache
			if(updateItemInCache(id, req.cacheUpdate.properties.map)) {
				success = true;
			}
			break;
		case 2: // Remove item from cache
			if(removeItemFromCache(id)) {
				success = true;
			}
			break;
		default:
			break;
	}
	printEnvironmentCache(); // Print the cache, for debugging properties
	res.success = success;
	return true;
}

/**
 * Print all the items in the cache to standard output
 **/
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

/**
 * Add a new item to the cache
 *
 * @param id The id of the item to add
 * @param properties the vector of properties
 *
 * @return true if item is added, false if there is already an item in the cache with the same id of the item to add
 **/
bool EnvironmentCache::addItemToCache(std::string id, const std::vector<environmentCommunicationMessages::KeyValuePair> &properties) {
	if(cache.count(id) == 0) {
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
 * @param id the id of the item to update
 * @param properties the vector with properties to update. If a property in this vector already exists in property list of the item it 
 * will be updated, else it will be added to the list of properties
 *
 * @return true if an item with the id is found in the cache, else false
 **/
bool EnvironmentCache::updateItemInCache(std::string id, const std::vector<environmentCommunicationMessages::KeyValuePair> &properties) {
	if(cache.count(id) == 1) {
		// Create iterator for the cache
		std::map< std::string, std::map<std::string, std::string> >::iterator cacheIt;
		// Let the iterator point to the Item with the id specified in the update
		cacheIt = cache.find(id);

		// Convert the vector of properties in the update message to a map
		std::map<std::string, std::string> propertiesMap;
		createMapFromVector(properties, propertiesMap);

		// Loop through all the properties that are specified in the update
		std::map<std::string, std::string>::iterator optionsIt;
		std::map<std::string, std::string>::iterator propertyIt;
		for(optionsIt = propertiesMap.begin(); optionsIt != propertiesMap.end(); optionsIt++) {
			// If the property is found update it, else add it to the list of properties
			if((*cacheIt).second.count((*optionsIt).first) == 1) {
				propertyIt = (*cacheIt).second.find((*optionsIt).first); // Create iterator to the property found
				(*propertyIt).second = (*optionsIt).second; // update it's value
				std::cout << "Property with key " << (*propertyIt).first << " in item with id " << (*cacheIt).first << " updated" << std::endl;
			} else {			
				(*cacheIt).second.insert(std::pair<std::string, std::string>((*optionsIt).first, (*optionsIt).second ));
				std::cout << "Property with key " << (*optionsIt).first << " added to item with id " << id << std::endl;
			}
		}
		return true;	
	}
	return false;
}

/**
 * Remove an item from the cache
 *
 * @param id the id of the item to delete
 *
 * @return true if item is deleted, false if no item with the id specified found
 **/
bool EnvironmentCache::removeItemFromCache(std::string id) {
	if(cache.count(id) == 1) {
		cache.erase(id);	
		std::cout << "Item with id " << id << " deleted from cache" << std::endl;
		return true;
	}
	return false;
}

/**
 * Convert a vector to a map
 *
 * @param properties the vector with KeyValuePair objects
 * @param the map where the keys and values of the objects in the vector is inserted to
 **/
void EnvironmentCache::createMapFromVector(const std::vector<environmentCommunicationMessages::KeyValuePair> &propertiesVector, std::map<std::string, std::string> &propertiesMap) {
	for(int i = 0; i < (int)propertiesVector.size(); i++) {
		propertiesMap.insert(std::pair<std::string, std::string>(propertiesVector[i].key, propertiesVector[i].value));
	}
}

/**
 * Create a Map message from a map with strings as key and strings as value
 *
 * @param properties the map to convert
 *
 * @return environmentCommunicationMessages::Map The Map message object
 **/
environmentCommunicationMessages::Map EnvironmentCache::createMapMessageFromProperties(std::map<std::string, std::string> &properties){
	std::map<std::string, std::string>::iterator propertiesIt;
	environmentCommunicationMessages::Map map;
	environmentCommunicationMessages::KeyValuePair prop;
	for(propertiesIt = properties.begin(); propertiesIt != properties.end(); propertiesIt++) {
		prop.key = (*propertiesIt).first;
		prop.value = (*propertiesIt).second;
		map.map.push_back(prop);
	}
	return map;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "EnvironmentCache");
	EnvironmentCache envCache;
	ros::spin();
	return 0;	
}