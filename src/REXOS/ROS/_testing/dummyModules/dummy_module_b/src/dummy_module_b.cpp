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

#include "dummy_module_b/dummy_module_b.h"

DummyModuleB::DummyModuleB(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, bool isShadow) :
		equipletName(equipletName),
		rexos_module::ActorModule(equipletName, moduleIdentifier, isSimulated, isShadow)
{
}

void DummyModuleB::run() {
	REXOS_INFO("running");
	ros::spin();
}

void DummyModuleB::onSetInstruction(const rexos_module::SetInstructionGoalConstPtr& goal) {
	REXOS_INFO("executing hardware step...");
	ros::Duration d(5.0);
	d.sleep();
	REXOS_INFO("done with executing hardware step");
	setInstructionActionServer.setSucceeded();
}

bool DummyModuleB::transitionInitialize() {
	REXOS_INFO("Initialize transition called");
	return true;
}

bool DummyModuleB::transitionDeinitialize() {
	REXOS_INFO("Deinitialize transition called");
	ros::shutdown();
	return true;
}


bool DummyModuleB::transitionSetup(){
	REXOS_INFO("Setup transition called");
	
	/*rexos_module::TransitionGoal goal;
	std::vector<rexos_module::RequiredMutation> requiredMutations;
	rexos_module::RequiredMutation requiredMutation;
	requiredMutation.mutation = "move";
	requiredMutation.isOptional = false;
	requiredMutations.push_back(requiredMutation);
	goal.requiredMutationsRequiredForNextPhase = requiredMutations;
	
	transitionActionClient.sendGoal(goal);
	transitionActionClient.waitForResult();*/
	
	REXOS_INFO("Continuing calibration");
	return true;
}
bool DummyModuleB::transitionShutdown(){
	REXOS_INFO("Shutdown transition called");
	return true;
}
bool DummyModuleB::transitionStart(){
	REXOS_INFO("Start transition called");
	return true;
}
bool DummyModuleB::transitionStop(){
	REXOS_INFO("Stop transition called");
	return true;
}

int main(int argc, char* argv[]) {
	if(argc < 5){
		REXOS_ERROR("Usage: dummy_module_b (--isSimulated | --isShadow) equipletName manufacturer typeNumber serialNumber");
		return -1;
	}
	
	bool isSimulated = false;
	bool isShadow = false;
	
	for (int i = 0; i < argc; i++) {
		std::string arg = argv[i];
		if (arg == "--isSimulated") {
			isSimulated = true;
		} else if (arg == "--isShadow") {
			isShadow = true;
			isSimulated = true;
		}
	}
	
	std::string equipletName = std::string(argv[argc - 4]);
	rexos_datatypes::ModuleIdentifier moduleIdentifier(argv[argc - 3], argv[argc - 2], argv[argc - 1]);
	
	// set up node namespace and name
	if(isShadow == true) {
		if(setenv("ROS_NAMESPACE", "shadow", 1) != 0) {
			REXOS_ERROR("Unable to set environment variable");
		}
	}
	std::string nodeName = equipletName + "_" + moduleIdentifier.getManufacturer() + "_" + 
			moduleIdentifier.getTypeNumber() + "_" + moduleIdentifier.getSerialNumber();
	ros::init(argc, argv, nodeName);
	
	DummyModuleB node(equipletName, moduleIdentifier, isSimulated, isShadow);
	
	ros::spin();
	return 0;
}
