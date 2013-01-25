/**
 * @file HardwareModuleProperties.h
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

#ifndef HWMODULEPROPERTIES_H
#define HWMODULEPROPERTIES_H

#include <string>
#include <iostream>
#include "rexos_mast/StateMachine.h"


	/**
	 * HardwareModuleProperties defines the the data that is for a module
	 **/
	class HardwareModuleProperties{
	public:
		HardwareModuleProperties();
		/**
		 * Create a new hardware module
		 * 
		 * @param id Unique identifier of the module
		 * @param type Defines the type of the module
		 * @param state The current state of this module
		 * @param actuator Is this module an actor
		 * @param needed Is this module needed for the current service
		 **/
		HardwareModuleProperties(int id, int type, rexos_mast::StateType state, bool actuator, bool needed):
			id(id), type(type), currentState(state), actuator(actuator), needed(needed), error(false){}
		/**
		 * @var int id
		 * The id of the module
		 **/
		int id;

		/** 
		 * @var int type
		 * The type of the module 
		 **/
		int type;

		/**
		 * @var rexos_mast::StateType currentState
		 *  The currentState of the module
		 **/
		rexos_mast::StateType currentState;

		/**
		 * @var bool actuator
		 * Defines if the hardware module is an actuator
		 **/
		bool actuator;

		/**
		 * @var bool needed
		 * defines if the hardware module is needed for the current service
		 **/
		bool needed;

		/**
		 * @var bool error
		 * shows if the modules is in an error state
		 **/
		bool error;

		/** 
		 * Print function for a module
		 **/
		friend std::ostream& operator<<(std::ostream& stream, HardwareModuleProperties &module){
			stream << "Id: " << module.id << ", current state: " << module.currentState << " actuator " << module.actuator << " Required for current service " << module.needed;
			return stream;
		}
};

#endif
