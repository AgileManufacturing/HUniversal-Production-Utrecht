/**
 * @file part_locator_node.cpp
 * @brief locates objects and rotates points.
 * @date Created: 2013-09-20
 *
 * @author Tommas Bakker
 *
 * @section LICENSE
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
 *
 **/

#pragma once

#include "ros/ros.h"

#include <rexos_module/Module.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_logger/rexos_logger.h>


class DummyModuleA : public rexos_module::Module {
public:
protected:
	std::string equipletName;
public:
	DummyModuleA(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, bool isShadow);
	void run();
	
	virtual bool transitionInitialize();
	virtual bool transitionDeinitialize();
	/**
	 * MAST transition from safe to standby is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionSetup();
	/**
	 * MAST transition from standby to safe is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionShutdown();
	/**
	 * MAST transition from standby to normal is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionStart();
	/**
	 * MAST transition from normal to standby is handled by this method. 
	 * see http://wiki.agilemanufacturing.nl/index.php/Vision_system#camera_control_node
	 **/
	virtual bool transitionStop();
private:
	ros::NodeHandle nodeHandle;
	ros::ServiceClient environmentCacheClient;
};
