
/**
 * @file EquipletNode.cpp
 * @brief Symbolizes an entire EquipletNode.
 * @date Created: 2012-10-12
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

#include <EquipletNode/EquipletNode.h>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <algorithm>

/**
 * Update the safetyState of the Equiplet
 * 
 * @param moduleTable The module containing all hardware modules
 **/
void EquipletNode::updateSafetyState() {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	rosMast::StateType newSafetyState = rosMast::safe;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if((*it).actuator && (*it).currentState > newSafetyState) {
			newSafetyState = (*it).currentState;
		}
	}
	safetyState = newSafetyState;
}

/**
 * Update the operation state of the Equiplet
 * 
 * @param moduleTable The module containing all hardware modules
 **/
void EquipletNode::updateOperationState() {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	bool operationStateSet = false;
	// first set the operation state to the highest state possible 
	rosMast::StateType newOperationState = rosMast::normal;

	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		/**
		 * Set the new operation state if the hardware module is an actor, required for
		 * the current service and if its state is lower than the new operation state as
		 * initialized above.
		 **/
		if((*it).actuator && (*it).needed) {
			newOperationState = std::min((*it).currentState, newOperationState);
			operationStateSet = true;
		}
	}
	/**
	 * If the operation state is not set, it means that there are no actor modules suited
	 * for the current service and so the operation state is equal to the lowest state possible
	 * the safe state.
	 **/
	if(!operationStateSet) {
		newOperationState = rosMast::safe;
	}
	operationState = newOperationState;
}

/**
 * Add a hardware module to the module table
 *
 * @param moduleTable The table containing the hardware module
 * @param module The hardware to add to the table
 * @return true if the module has a unique name, otherwise false
 **/
bool EquipletNode::addHardwareModule(Mast::HardwareModuleProperties module) {
	/**
	 * The use of the name to uniquely identify a hardware module is a temporary,
	 * solution. This will probably be changed when the module database is implemented.
	 **/
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if(module.id == (*it).id) {
			return false;
		}
	}

	/**
	 * Create the string that is used to start another ROS node
	 **/ 
	std::pair< std::string, std::string > packageNodeName = modulePackageNodeMap[module.type];
	stringstream ss (stringstream::in | stringstream::out);
	ss << "rosrun " << packageNodeName.first << " " << packageNodeName.second
	<< " __name:=" << packageNodeName.second << "" << module.id;

	int pid = -1;
	switch(pid = fork()) {
		case 0:
			fclose(stderr);
			fclose(stdout);
			fclose(stdin);
			execl("/bin/sh", "/bin/sh", "-c", ss.str().c_str(), NULL);
		case -1: 
			std::cerr << "Cannot start node for hardwaremodule " << module.id << std::endl;
			return false;
		default:
			break; 
	}

	/**
	 * Add the module to the table and update the safety state and operation state	
	 **/
	moduleTable->push_back(module);
	updateSafetyState();
	updateOperationState(); 
	return true;
}

/**
 * Remove a hardware module from the module table
 *
 * @param moduleTable The table containing the hardware modules
 * @param name The name that is used to uniquely identify the hardware module that needs to be removed
 *
 * @return true if the hardware module is removed, false if the module could not be found in the table
 **/
bool EquipletNode::removeHardwareModule(int id) {
	/**
	 * The use of the name to uniquely identify a hardware module is a temporary,
	 * solution. This will probably be changed when the module database is implemented.
	 **/
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if((*it).id == id) {
			moduleTable->erase(it);
			updateSafetyState();
			updateOperationState();
			return true;
		}
	}	
	return false;
}

/**
 * Print all hardware modules in the table
 *
 * @param moduleTable The module table containing the hardware modules
 **/
void EquipletNode::printHardwareModules() {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		std::cout << *it << std::endl;
	}
}

/**
 * Read from the blackboard, store the messages in postItBox
 **/
void EquipletNode::readFromBlackboard() {
    PostItBox_Filter * f = postItBox->mutable_filter();
    PostItBox * received = new PostItBox();
    f->set_filtername("PostItFilter");
	bbUtils->readFromBlackboard(postItBox);
	std::cout << "Number of postIts: " << received->postits_size() << std::endl;
}

/**
 * Update the state of a module in the module table. Also automatically updates the operationState and
 * safe state of the Equiplet
 *
 * @param moduleID the id of the module
 * @param state the new state of the module 
 * 
 * @return true if the module is found and the state is updated, false if the module is not found in module table
 **/
bool EquipletNode::updateModuleState(int moduleID, rosMast::StateType state) {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if((*it).id == moduleID) {
			(*it).currentState = state;
			updateSafetyState();
			updateOperationState();
			return true;
		}
	}
	return false;
}	