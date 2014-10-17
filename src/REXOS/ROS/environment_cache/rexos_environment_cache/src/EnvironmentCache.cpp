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

#include <rexos_environment_cache/EnvironmentCache.h>
#include <iostream>
#include <jsoncpp/json/writer.h>
#include <jsoncpp/json/reader.h>
#include <boost/algorithm/string.hpp>

#define ENVIRONMENT_CACHE_DEBUG

namespace rexos_environment_cache {
	std::string EnvironmentCache::printEnvironmentCache() {
		Json::StyledWriter writer;
		std::stringstream ss;
		ss << "{" << std::endl;
		for(auto it = cache.begin(); it != cache.end(); it++) {
			ss << it->first << " : " << writer.write(it->second);
		}
		ss << "}" << std::endl;
		return ss.str();
	}
	
	Json::Value EnvironmentCache::getItemDataInCache(std::string identifier, std::vector<std::string> paths) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Getting data for " << identifier);
#endif
		Json::Value output;
		
		for(int i = 0; i < paths.size(); i++) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Processing path " << paths[i]);
#endif
			std::vector<std::string> pathElements;
			boost::split(pathElements, paths[i], boost::is_any_of("\t/"));
			
			Json::Value currentNode = getJsonNodeForItem(identifier);
			bool pathExists = true;
			for(int j = 0; j < pathElements.size(); j++) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Processing pathElement " << paths[i]);
#endif
				if(currentNode.isMember(pathElements[j]) == false) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Path does not exist");
#endif
					pathExists = false;
					break;
				} else {
					currentNode = currentNode[pathElements[j]];
				}
			}
			
			if(pathExists == true) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Path existed, value: " << currentNode);
#endif
				Json::Value currentOutputNode = currentNode;
				for(int j = pathElements.size() - 1; j >= 0; j--) {
					Json::Value newNode;
					newNode[pathElements[j]] = currentOutputNode;
					currentOutputNode = newNode;
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Current output node: " << currentOutputNode);
#endif
				}
				mergeJsonNode(currentOutputNode, output);
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Merge result: " << output);
#endif
			}
			
			
		}
		return output;
	}
	void EnvironmentCache::setItemDataInCache(std::string identifier, Json::Value data) {
		Json::Value& currentNode = cache[identifier];
		mergeJsonNode(data, currentNode);
	}
	void EnvironmentCache::removeDataOfItemFromCache(std::string identifier, std::vector<std::string> paths) {
		for(int i = 0; i < paths.size(); i++) {
			std::vector<std::string> pathElements;
			boost::split(pathElements, paths[i], boost::is_any_of("\t/"));
			
			Json::Value& currentNode = getJsonNodeForItem(identifier);
			bool pathExists = true;
			for(int j = 0; j < pathElements.size() - 1; j++) {
				if(currentNode.isMember(pathElements[j]) == false) {
					pathExists = false;
					break;
				} else {
					currentNode = currentNode[pathElements[j]];
				}
			}
			currentNode.removeMember(pathElements[pathElements.size() - 1]);
		}
		if(getJsonNodeForItem(identifier) == Json::Value(Json::ValueType::objectValue)) {
			// this item no longer has any properties in the cache and thus should be removed
			cache.erase(identifier);
		}
	}

	Json::Value& EnvironmentCache::getJsonNodeForItem(std::string identifier) {
		if(cache.find(identifier) != cache.end()) return cache[identifier];
		else throw std::runtime_error("Item does not exist");
	}

	void EnvironmentCache::mergeJsonNode(Json::Value& toBeMerged, Json::Value& mergeTarget) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Merging json nodes, toBeMerged: " << toBeMerged << " mergeTerget: " << mergeTarget);
#endif
		std::vector<std::string> memberNames = toBeMerged.getMemberNames();
		
		for(int i = 0; i < memberNames.size(); i++) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Merging member " << memberNames[i]);
#endif
			if(toBeMerged[memberNames[i]].type() == Json::ValueType::objectValue) {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Member is JsonObject");
#endif
				mergeJsonNode(toBeMerged[memberNames[i]], mergeTarget[memberNames[i]]);
			} else {
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Member is not JsonObject");
#endif
				mergeTarget[memberNames[i]] = toBeMerged[memberNames[i]];
			}
		}
#ifdef ENVIRONMENT_CACHE_DEBUG
		ROS_DEBUG_STREAM("Merge result " << mergeTarget);
#endif
	}
	std::map<std::string, Json::Value> EnvironmentCache::getCache() {
		return cache;
	}
}