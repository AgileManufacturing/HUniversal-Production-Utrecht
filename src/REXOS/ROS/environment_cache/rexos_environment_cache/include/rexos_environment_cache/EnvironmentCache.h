/**
 * @file EnvironmentCache.h
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

#pragma once 

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"
#include <map>
#include <iostream>
#include <jsoncpp/json/value.h>

namespace rexos_environment_cache{
	/**
	 * This class represents the environment cache
	 **/
	class EnvironmentCache{
	public:
		std::string printEnvironmentCache();
		
		Json::Value getItemDataInCache(std::string identifier, std::vector<std::string> paths);
		void setItemDataInCache(std::string identifier, Json::Value data);
		void removeDataOfItemFromCache(std::string identifier, std::vector<std::string> paths);
		
	private:
		Json::Value& getJsonNodeForItem(std::string identifier);
		void mergeJsonNode(Json::Value& toBeMerged, Json::Value& mergeTarget);
		
		/**
		 * @var The cache used for actually storing the data. The key is the identifier of the item for which the data is stored. 
		 * The value is the JSON node containing all the data.
		 **/
		std::map<std::string, Json::Value> cache;
	public:
		std::map<std::string, Json::Value> getCache();
	};
}
