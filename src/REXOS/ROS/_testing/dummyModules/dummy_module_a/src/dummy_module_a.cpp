/**
 * @file part_locator_node.cpp
 * @brief locates objects and rotates points.
 * @date Created: 2013-09-20
 *
 * @author Garik hakopian
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
#include "ros/ros.h"

#include "dummy_module_a/dummy_module_a.h"
#include <actionlib/client/simple_action_client.h>



DummyModuleA::DummyModuleA(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier):
		equipletName(equipletName),
		rexos_knowledge_database::Module(moduleIdentifier),
		rexos_statemachine::ModuleStateMachine(equipletName, moduleIdentifier, false)
{
}

void DummyModuleA::run() {
	REXOS_INFO("running");
	ros::spin();
}
bool DummyModuleA::transitionInitialize() {
	REXOS_INFO("Initialize transition called");
	return true;
}

bool DummyModuleA::transitionDeinitialize() {
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}


bool DummyModuleA::transitionSetup(){
	REXOS_INFO("Setup transition called");
	
	ros::Duration(5.0).sleep();
	
	rexos_statemachine::TransitionGoal goal;
	goal.gainedSupportedMutations.push_back("move");
	
	transitionActionClient.sendGoal(goal);
	REXOS_INFO("done");
	return true;
}
bool DummyModuleA::transitionShutdown(){
	REXOS_INFO("Shutdown transition called");
	return true;
}
bool DummyModuleA::transitionStart(){
	REXOS_INFO("Start transition called");
	return true;
}
bool DummyModuleA::transitionStop(){
	REXOS_INFO("Stop transition called");
	return true;
}

int main(int argc, char* argv[]) {
	if(argc < 5){
		REXOS_ERROR("Usage: dummy_module_a equipletId, manufacturer, typeNumber, serialNumber");
		return -1;
	}
	
	std::string equipletName = argv[1];
	rexos_knowledge_database::ModuleIdentifier moduleIdentifier = rexos_knowledge_database::ModuleIdentifier(argv[2], argv[3], argv[4]);

	REXOS_INFO("Creating DummyModuleA");
	ros::init(argc, argv, std::string("dummy_module_a") + moduleIdentifier.getManufacturer() + "_" + 
			 moduleIdentifier.getTypeNumber() + "_" + 
			 moduleIdentifier.getSerialNumber());
	DummyModuleA node(equipletName, moduleIdentifier);
	node.run();
	
	return 0;
}
