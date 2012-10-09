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

#define TOP_CAMERA "TopCamera"
#define BOTTOM_CAMERA "BottomCamera"
#define GRIPPER "Gripper"

typedef std::vector<Mast::HardwareModuleProperties> moduleList;

/**
 * Add a hardware module to the module table
 *
 * @param moduleTable The table containing the hardware module
 * @param module The hardware to add to the table
 * @return true if the module has a unique name, otherwise false
 **/
bool addHardwareModule(moduleList *moduleTable, const Mast::HardwareModuleProperties module) {
	/**
	 * The use of the name to uniquely identify a hardware module is a temporary,
	 * solution. This will probably be changed when the module database is implemented.
	 **/
	moduleList::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if(module.name.compare(((*it).name)) == 0) {
			return false;
		}
	}	
	moduleTable->push_back(module);
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
bool removeHardwareModule(moduleList *moduleTable, const std::string& name) {
	/**
	 * The use of the name to uniquely identify a hardware module is a temporary,
	 * solution. This will probably be changed when the module database is implemented.
	 **/
	moduleList::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		if((*it).name.compare(name) == 0) {
			moduleTable->erase(it);
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
void printHardwareModules(moduleList *moduleTable) {
	moduleList::iterator it;
	for(it = moduleTable->begin(); it < moduleTable->end(); it++) {
		std::cout << *it << std::endl;
	}
}

int main(int argc, char **argv) {
	/**
	 * The safety state of the Equiplet. This is equal 
	 * to the highest state of the actuator modules
	 **/
	Mast::state safetyState;


	// The table that holds the information about all hardware modules
	moduleList *moduleTable = new moduleList();
	Mast::HardwareModuleProperties topCamera(TOP_CAMERA, Mast::safe, false, false);
	Mast::HardwareModuleProperties bottomCamera(BOTTOM_CAMERA, Mast::safe, false, false);
	Mast::HardwareModuleProperties gripper(GRIPPER, Mast::safe, true, false);

	addHardwareModule(moduleTable, topCamera);
	addHardwareModule(moduleTable, bottomCamera);
	addHardwareModule(moduleTable, gripper);

	printHardwareModules(moduleTable);

	topCamera.currentState = Mast::normal;
	printHardwareModules(moduleTable);

	removeHardwareModule(moduleTable, BOTTOM_CAMERA);

	printHardwareModules(moduleTable);
	removeHardwareModule(moduleTable, GRIPPER);
	printHardwareModules(moduleTable);

	delete moduleTable;

	return 0;
}