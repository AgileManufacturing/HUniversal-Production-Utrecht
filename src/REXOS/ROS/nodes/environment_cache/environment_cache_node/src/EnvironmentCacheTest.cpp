/**
 * @file EnvironmentCacheTest.cpp
 * @brief The EnvironmentCache definition
 * @date Created: 2014-10-03
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
#include <jsoncpp/json/writer.h>







/** 
 * Main that runs the tests
 **/
int main(int argc, char **argv){
	rexos_environment_cache::EnvironmentCache cache;
	
	Json::StyledWriter writer;
	Json::Value result;
	
	Json::Value partA;
	partA["location"]["x"] = -30.5;
	partA["location"]["y"] = 24.1;
	partA["rotation"]["z"] = -0.12;
	partA["weight"] = 813;
	cache.setItemDataInCache("partA", partA);
	if(cache.getCache().at("partA") != partA) {
		ROS_ERROR_STREAM("Test 1 failed: \n" << cache.printEnvironmentCache());
		return -1;
	}
	
	Json::Value partB;
	partB["location"]["z"] = -76.1;
	partB["rotation"]["z"] = 1.3;
	partB["temperature"] = 300;
	cache.setItemDataInCache("partB", partB);
	if(cache.getCache().at("partB") != partB) {
		ROS_ERROR_STREAM("Test 2 failed: \n" << cache.printEnvironmentCache());
		return -2;
	}
	
	partB["location"]["z"] = 0;
	partB["rotation"]["z"] = 0;
	partB["temperature"] = 0;
	cache.setItemDataInCache("partB", partB);
	if(cache.getCache().at("partB") != partB) {
		ROS_ERROR_STREAM("Test 3 failed: \n" << cache.printEnvironmentCache());
		return -3;
	}
	
	std::vector<std::string> partAGetPaths;
	partAGetPaths.push_back("location/x");
	result = cache.getItemDataInCache("partA", partAGetPaths);
	if(result["location"]["x"] != partA["location"]["x"]) {
		ROS_ERROR_STREAM("Test 4 failed: \n" << writer.write(result));
		return -4;
	}
	
	std::vector<std::string> partARemovePaths;
	partARemovePaths.push_back("location");
	partARemovePaths.push_back("weight");
	partARemovePaths.push_back("rotation/z");
	cache.removeDataOfItemFromCache("partA", partARemovePaths);
	auto interalCache = cache.getCache();
	if(interalCache.find("partA") != interalCache.end()) {
		ROS_ERROR_STREAM("Test 5 failed: \n" << cache.printEnvironmentCache());
		return -5;
	}
	
	return 0;
}