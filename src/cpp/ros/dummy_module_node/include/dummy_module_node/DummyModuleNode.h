/**
 * @file DummyModuleNode.h
 * @brief A dummy module!
 * @date Created: 2013-03-13
 *
 * @author Arjen van Zanten
 * @author Ammar Abdulamir
 *
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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

#ifndef DUMMYMODULENODE_H
#define DUMMYMODULENODE_H

#include "ros/ros.h"
#include "rexos_std_srvs/Module.h"

#include <rexos_utilities/Utilities.h>
#include <rexos_statemachine/ModuleStateMachine.h>

// GCC system header to suppress libjson warnings
#pragma GCC system_header
#include <libjson/libjson.h>
// ---------------------------------------------

class DummyModuleNode : public rexos_statemachine::ModuleStateMachine{
public:
	DummyModuleNode(int equipletID, int moduleID);
	virtual ~DummyModuleNode();

	// services
	bool outputJSON(rexos_std_srvs::Module::Request &req, rexos_std_srvs::Module::Response &res);

private:
	virtual void transitionSetup();
	virtual void transitionShutdown();
	virtual void transitionStart();
	virtual void transitionStop();
	
	/**
	 * @var std::string nodeName
	 * The node's name.
	 **/
	 std::string nodeName;

	/**
	 * @var ros::ServiceServer outputJSON
	 * Service for outputting coordinates
	 **/
	ros::ServiceServer outputJSONService;
};

#endif
