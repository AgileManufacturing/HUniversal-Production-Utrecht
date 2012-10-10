/**
* @file EquipletNode.cpp
* @brief Symbolizes an entire Equiplet.
* @date Created: 2012-10-09
*
* @author Dennis Koole
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
**/

#include <vector>
#include <Mast/HardwareModuleProperties.h>
#include <algorithm>

#define TOP_CAMERA "TopCamera"
#define BOTTOM_CAMERA "BottomCamera"
#define DELTAROBOT "DeltaRobot"
#define GRIPPER2 "Gripper2"

/**
 * The safety state of the Equiplet. This is equal 
 * to the highest state of the actor modules
 **/
Mast::state safetyState;

/**
 * The minimal operation state is equal to the lowest state of 
 * all modules that are actors
 **/
Mast::state operationState;

/**
 * Update the safetyState of the Equiplet
 * 
 * @param moduleTable The module containing all hardware modules
 **/
void updateSafetyState(std::vector<Mast::HardwareModuleProperties> *moduleTable) {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	Mast::state newSafetyState = Mast::safe;
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
void updateOperationState(std::vector<Mast::HardwareModuleProperties> *moduleTable) {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	bool operationStateSet = false;
	// first set the operation state to the highest state possible 
	Mast::state newOperationState = Mast::normal;

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
		newOperationState = Mast::safe;
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
bool addHardwareModule(std::vector<Mast::HardwareModuleProperties> *moduleTable, const Mast::HardwareModuleProperties module) {
	/**
	 * The use of the name to uniquely identify a hardware module is a temporary,
	 * solution. This will probably be changed when the module database is implemented.
	 **/
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if(module.name.compare(((*it).name)) == 0) {
			return false;
		}
	}	
	moduleTable->push_back(module);
	updateSafetyState(moduleTable);
	updateOperationState(moduleTable);
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
bool removeHardwareModule(std::vector<Mast::HardwareModuleProperties> *moduleTable, const std::string& name) {
	/**
	 * The use of the name to uniquely identify a hardware module is a temporary,
	 * solution. This will probably be changed when the module database is implemented.
	 **/
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if((*it).name.compare(name) == 0) {
			moduleTable->erase(it);
			updateSafetyState(moduleTable);
			updateOperationState(moduleTable);
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
void printHardwareModules(std::vector<Mast::HardwareModuleProperties> *moduleTable) {
	std::vector<Mast::HardwareModuleProperties>::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		std::cout << *it << std::endl;
	}
}

int main(int argc, char **argv) {
	// The table that holds the information about all hardware modules
	std::vector<Mast::HardwareModuleProperties> *moduleTable = new std::vector<Mast::HardwareModuleProperties>();
	Mast::HardwareModuleProperties topCamera(TOP_CAMERA, Mast::safe, false, false);
	Mast::HardwareModuleProperties bottomCamera(BOTTOM_CAMERA, Mast::safe, false, false);
	Mast::HardwareModuleProperties deltarobot(DELTAROBOT, Mast::start, true, false);
	Mast::HardwareModuleProperties gripper2(GRIPPER2, Mast::standby, true, true);

	// Add the three hardware modules
	std::cout << "Add four hardware modules..." << std::endl;
	addHardwareModule(moduleTable, topCamera);
	addHardwareModule(moduleTable, bottomCamera);
	addHardwareModule(moduleTable, deltarobot);
	addHardwareModule(moduleTable, gripper2);

	printHardwareModules(moduleTable);

	std::cout << std::endl;
	std::cout << "safety state: " << safetyState << std::endl;
	std::cout << "operation state: " << operationState << std::endl; 

	std::cout << std::endl;
	std::cout << "Remove the hardware module " << GRIPPER2 << std::endl;
	removeHardwareModule(moduleTable, GRIPPER2);

	printHardwareModules(moduleTable);

	std::cout << std::endl;
	std::cout << "safety state: " << safetyState << std::endl;
	std::cout << "operation state: " << operationState << std::endl; 

	// delete the vector
	delete moduleTable;

	return 0;
}