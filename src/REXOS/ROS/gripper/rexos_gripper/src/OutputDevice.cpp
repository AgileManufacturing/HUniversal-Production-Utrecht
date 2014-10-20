/**
* @file OutputDevice.cpp
* @brief Output device interface
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

#include <rexos_gripper/OutputDevice.h>

#include "ros/ros.h"

namespace rexos_gripper {
	/**
	 * Constructor for OutputDevice
	 *
	 * @param ioController Pointer to an established modbus connection.
	 * @param address Register address that contains the device boolean.
	 * @param pin The pin (bit) that is connected to the device.
	 **/
	OutputDevice::OutputDevice(Json::Value node) {
		readJSONNode(node);
		
		ioController = new InputOutputController(node);
	}
	void OutputDevice::readJSONNode(const Json::Value node) {
		address = node["modbusAddress"].asInt();
		REXOS_INFO_STREAM("found modbusAddress " << address);
		pin = node["modbusDevicePin"].asInt();
		REXOS_INFO_STREAM("found modbusDevicePin " << pin);
	}
	
	/**
	 * Virtual destructor for extensions of OutputDevice.
	 */
	OutputDevice::~OutputDevice(){
	}

	/**
	 * Turns on the pin of the output device
	 **/
	void OutputDevice::enable(){
		std::lock_guard<std::mutex> guard(mu);
		ioController->pinHigh(address, pin);
	}

	/**
	 * Turns off the pin of the output device
	 **/
	void OutputDevice::disable(){
		std::lock_guard<std::mutex> guard(mu);
		ioController->pinLow(address, pin);
	}
}
