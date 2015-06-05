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

#ifndef ENVIRONMENTCACHE_H
#define ENVIRONMENTCACHE_H

#include "ros/ros.h"
#include "rexos_logger/rexos_logger.h"
#include <rexos_environment_cache/EnvironmentCache.h>
#include <environment_cache/getData.h>
#include <environment_cache/setData.h>
#include <environment_cache/removeData.h>
#include <map>
#include <jsoncpp/json/value.h>

/**
 * This class represents the environment cache
 **/
class EnvironmentCache{
public:
	EnvironmentCache(std::string equipletName);
	bool getData(environment_cache::getData::Request& req, environment_cache::getData::Response& res);
	bool setData(environment_cache::setData::Request& req, environment_cache::setData::Response& res);
	bool removeData(environment_cache::removeData::Request& req, environment_cache::removeData::Response& res);
	
protected:
	std::string equipletName;
	/**
	 * @var The enviroment cache
	 **/
	rexos_environment_cache::EnvironmentCache cache;
	/**
	 * @var The Ros nodehandle
	 **/
	ros::NodeHandle nh;
	/**
	 * @var The ROS service server for the getData service
	 **/
	ros::ServiceServer getDataServiceServer;
	/**
	 * @var The ROS service server for the setData service
	 **/
	ros::ServiceServer setDataServiceServer;
	/**
	 * @var The ROS service server for the removeData service
	 **/
	ros::ServiceServer removeDataServiceServer;
};
#endif
