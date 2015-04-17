/**
 * @file ModbusController.h
 * @brief Wrapper for libmodbus with some extra functionality.
 * @date Created: 2012-10-01
 *
 * @author 1.0 Lukas Vermond
 * @author 1.0 Kasper van Nieuwland
 * @author 1.1 Koen Braham
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

#pragma once

extern "C"{
	#include <modbus/modbus.h>
}

#include <stdint.h>
#include <string>
#include <boost/thread.hpp>
#include <map>

#include <fstream>
#include <iostream>
#include <string>

#include <rexos_io/ModbusInputOutputController.h>

/**
 * @cond HIDE_FROM_DOXYGEN
 * Turn on for modbus logging
 * Parameter is the logfile location. Change before enabling the log!
 *
 * #define MODBUS_LOGGING "/home/agileman/modbus.log"
 * @endcond
 **/

namespace rexos_io {
	/**
	 * Wrapper class for libmodbus with some extra functionality.
	 **/
	class RtuModbusInputOutputController : public ModbusInputOutputController {
	public:
		RtuModbusInputOutputController();
		~RtuModbusInputOutputController(void);
		
		void writeU16		(uint16_t slave, uint16_t address, uint16_t data, bool useShadow = false);
		void writeU32		(uint16_t slave, uint16_t address, uint32_t data, bool useShadow = false);
		uint16_t readU16	(uint16_t slave, uint16_t address, bool useShadow = false);
		uint32_t readU32	(uint16_t slave, uint16_t address, bool useShadow = false);
		void writeU16		(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length);
		void readU16		(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length);

	private:
		void writeShadowU16		(uint16_t slave, uint16_t address, uint16_t value);
		void writeShadowU32		(uint16_t slave, uint16_t address, uint32_t value);
		uint16_t readShadowU16	(uint16_t slave, uint16_t address);
		uint32_t readShadowU32	(uint16_t slave, uint16_t address);
	};
}
