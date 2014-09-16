/**
 * @file InputOutputController.cpp
 * @brief Manager for the IO ports
 * @date Created: 2012-10-16
 *
 * @author Koen Braham
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

#include <rexos_gripper/InputOutputController.h>
#include <modbus/modbus.h>

#include "ros/ros.h"

namespace rexos_gripper {
	/**
	 * Constructor for InputOutputController
	 *
	 * @param modbus Pointer to an established modbus connection.
	 */
	InputOutputController::InputOutputController(JSONNode node) {
		readJSONNode(node);

		REXOS_INFO("[DEBUG] Opening modbus connection");
		
		modbusIO = modbus_new_tcp(modbusIp.c_str(), modbusPort);
		if(modbusIO == NULL){
			throw std::runtime_error("Unable to allocate libmodbus context");
		}
		if(modbus_connect(modbusIO) == -1) {
			throw std::runtime_error("Modbus connection to IO controller failed");
		}
		assert(modbusIO != NULL);
		
		modbus = new rexos_modbus::ModbusController(modbusIO);
	}
	void InputOutputController::readJSONNode(const JSONNode node) {
		for(JSONNode::const_iterator it = node.begin(); it != node.end(); it++) {
			if(it->name() == "modbusIp"){
				modbusIp = it->as_string();
				REXOS_INFO_STREAM("found modbusIp " << modbusIp);
			} else if(it->name() == "modbusPort"){
				modbusPort = it->as_int();
				REXOS_INFO_STREAM("found modbusPort " << modbusPort);
				
			} else {
				// some other property, ignore it
			}
		}
	}
	
	/*InputOutputController::InputOutputController(rexos_modbus::ModbusController* modbus) :
			modbus(modbus) {
	}*/

	/**
	 * Sets a pin (bit) in a register high (active?)
	 *
	 * @param address Register address in controller
	 * @param pin The pin to be set
	 **/
	void InputOutputController::pinHigh(uint32_t address, uint8_t pin) {
		uint16_t value = modbus->readU16(MODBUS_TCP_SLAVE, address);
		value |= (1 << pin);
		modbus->writeU16(MODBUS_TCP_SLAVE, address, value, false);
	}

	/**
	 * Clears a pin (bit) in an given register.
	 *
	 * @param address Register address in controller
	 * @param pin The pin to be cleared
	 **/
	void InputOutputController::pinLow(uint32_t address, uint8_t pin) {
		uint16_t value = modbus->readU16(MODBUS_TCP_SLAVE, address);
		value &= (0xFFFF ^ (1 << (pin)));
		modbus->writeU16(MODBUS_TCP_SLAVE, address, value, false);
	}
}
