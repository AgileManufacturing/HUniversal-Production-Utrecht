/**
* @file ModuleProperties.cpp
* @brief Holds all properties of a hardware module. 
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

#include <string>
#include <iostream>
#include "rosMast/StateMachine.h"

namespace Mast {
	class HardwareModuleProperties {
	public:
		HardwareModuleProperties();
		/**
		 * Create a new hardware module
		 * 
		 * @param id unique identifier of the module
		 * @param type type of the module
		 * @param state The current state of this module
		 * @param actuator Is this module an actor
		 * @param needed Is this module needed for the current service
		 **/
		HardwareModuleProperties(int id, int type, rosMast::StateType state, bool actuator, bool needed):
			id(id), type(type), currentState(state), actuator(actuator), needed(needed), error(false){}
		/**
		 * The use of a name for a module is a temporary solution. 
		 * This will probably be changed when the module database is implemented.
		 **/
		int id;
		int type;
		rosMast::StateType currentState;
		bool actuator;		
		/**
		 * defines if the hardware module is needed for the current service
		 **/
		bool needed;
		bool error;
		friend std::ostream& operator<<(std::ostream& stream, HardwareModuleProperties &module) {
			stream << "Id: " << module.id << ", current state: " << module.currentState << " actuator " << module.actuator << " Required for current service " << module.needed;
			return stream;
		}
	};
}