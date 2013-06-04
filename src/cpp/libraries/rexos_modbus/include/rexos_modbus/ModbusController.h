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

/**
 * @cond HIDE_FROM_DOXYGEN
 * Turn on for modbus logging
 * Parameter is the logfile location. Change before enabling the log!
 *
 * #define MODBUS_LOGGING "/home/agileman/modbus.log"
 * @endcond
 **/

namespace rexos_modbus{
	/**
	 * Wrapper class for libmodbus with some extra functionality.
	 **/
	class ModbusController{
	public:
		ModbusController(modbus_t* context);
		~ModbusController(void);

		void writeU16(uint16_t slave, uint16_t address, uint16_t data, bool useShadow = false);
		void writeU16(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length);
		void writeU32(uint16_t slave, uint16_t address, uint32_t data, bool useShadow = false);
		uint16_t readU16(uint16_t slave, uint16_t address);
		void readU16(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length);
		uint32_t readU32(uint16_t slave, uint16_t address);

	private:
		enum{
			/**
			 * modbus timeout error code
			 **/
			MODBUS_ERRNO_TIMEOUT = 0x6E,

			/**
			 * The interval between writing on the modbus in unicast mode milliseconds.
			 **/
			WRITE_INTERVAL_UNICAST = 8,

			/**
			 * The interval between writing on the modbus in broadcast mode in milliseconds.
			 **/
			WRITE_INTERVAL_BROADCAST = 16,

			/**
			 * Timeout for bytes in a response. This timeout will occur when a message is delayed while being send.
			 * Value in microseconds.
			 **/
			TIMEOUT_BYTE = 150000,

			/**
			 * Timeout for responses. This timeout will occur before a message is send.
			 * Value in microseconds.
			 **/
			TIMEOUT_RESPONE = 150000,
		};

		/**
		 * @var modbus_t* context
		 * A pointer to the modbus interface.
		 **/
		modbus_t* context;

		/**
		 * @var long nextWriteTime
		 * Next time is used for synchronisation on the modbus interface.
		 * Some devices require a certain wait time before a next request can be processed.
		 **/
		long nextWriteTime;

		/**
		 * Typedef for a shadowMap registers spread over multiple slaves.
		 * Key is slave address. 64bit for multiple slaves.
		 **/
		typedef std::map<uint64_t, uint16_t> ShadowMap;

		/**
		 * @var ShadowMap shadowRegisters
		 * Actual shadowmap instance.
		 * @see ShadowMap
		 **/
		ShadowMap shadowRegisters;

		#ifdef MODBUS_LOGGING
			/**
			 * @var std::ofstream logFile
			 * Logfile used for debugging the modbus communication.
			 **/
			std::ofstream logFile;
		#endif

		void wait(void);

		uint64_t getShadowAddress(uint16_t slave, uint16_t address);
		bool getShadow(uint16_t slave, uint32_t address, uint16_t& outValue);
		void setShadow(uint16_t slave, uint32_t address, uint16_t value);
		void setShadow32(uint16_t slave, uint32_t address, uint32_t value);
	};
}
