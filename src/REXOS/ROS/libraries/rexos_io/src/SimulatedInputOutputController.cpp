/**
 * @file StepperMotor.cpp
 * @brief Steppermotor driver.
 * @date Created: 2012-10-02
 *
 * @author Koen Braham
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

#include <rexos_io/SimulatedInputOutputController.h>
#include <rexos_io/InputOutputException.h>
#include <rexos_io/writeU16.h>
#include <rexos_io/readU16.h>

#include <ros/ros.h>


namespace rexos_io {
	SimulatedInputOutputController::SimulatedInputOutputController(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier):
		InputOutputControllerInterface(), equipletName(equipletName), identifier(identifier), nodeHandle() {
		servicePath = equipletName + "/" + identifier.getManufacturer() + "/" + 
				identifier.getTypeNumber() + "/" + identifier.getSerialNumber() + "/";
		
	}

	SimulatedInputOutputController::~SimulatedInputOutputController() {
	}
	void SimulatedInputOutputController::writeU16(uint16_t firstAddress, uint16_t* data, unsigned int length) {
		for(uint i = 0; i < length; i++) {
			std::string clientPath = servicePath + "/" + boost::lexical_cast<std::string>(firstAddress + i) + "/write/";
			ros::ServiceClient client = nodeHandle.serviceClient<rexos_io::writeU16>(clientPath);
			if(client.exists() == true) {
				rexos_io::writeU16 call;
				call.request.value = data[i];
				client.call(call);
			} else {
				throw rexos_io::InputOutputException("The serviceServer at " + clientPath + " was not availiable");
			}
		}
	}
	void SimulatedInputOutputController::readU16(uint16_t firstAddress, uint16_t* data, unsigned int length) {
		for(uint i = 0; i < length; i++) {
			std::string clientPath = servicePath + "/" + boost::lexical_cast<std::string>(firstAddress + i) + "/read/";
			ros::ServiceClient client = nodeHandle.serviceClient<rexos_io::readU16>(clientPath);
			if(client.exists() == true) {
				rexos_io::readU16 call;
				client.call(call);
				data[i] = call.response.value;
			} else {
				throw rexos_io::InputOutputException("The serviceServer at " + clientPath + " was not availiable");
			}
		}
	}
	
	void SimulatedInputOutputController::writeShadowU16(uint16_t address, uint16_t value) {
		// shadowRegistry entry will be created if slave did not exist yet
		InputOutputControllerInterface::writeShadowU16(address, value, shadowRegistry);
	}
	/*void SimulatedInputOutputController::writeShadowU32(uint16_t address, uint32_t value) {
		// shadowRegistry entry will be created if slave did not exist yet
		InputOutputControllerInterface::writeShadowU32(address, value, shadowRegistry);
	}*/
	uint16_t SimulatedInputOutputController::readShadowU16(uint16_t address) {
		// shadowRegistry entry will be created if slave did not exist yet
		return InputOutputControllerInterface::readShadowU16(address, shadowRegistry);
	}
	/*uint32_t SimulatedInputOutputController::readShadowU32(uint16_t address) {
		// shadowRegistry entry will be created if slave did not exist yet
		return InputOutputControllerInterface::readShadowU32(address, shadowRegistry);
	}*/
}
