/**
 * @file ModbusException.h
 * @brief Exception thrown if an error ocures during modbus actions.
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

#include <stdexcept>
#include <string>
#include <sstream>
#include <cerrno>

#include <modbus/modbus.h>

namespace rexos_modbus{
	/** 
	 * Exception calss for modbus communication.
	 **/
	class ModbusException : public std::runtime_error{
	private:
		/**
		 * @var int errorCode
		 * The error code is set by libmodbus5.
		 **/
		const int errorCode;

		/**
		 * @var std::string message
		 * Modbus error string (obtained using modbus_strerror).
		 */
		std::string message;

	public:
		/**
		 * Constructor of the modbus exception
		 * Retrieves an error string from the modbus library.
		 **/
        ModbusException(void) : std::runtime_error(""), errorCode(errno), message(""){
			std::stringstream stream;
			stream << "modbus error[" << errorCode << "]: " << modbus_strerror(errorCode);
			message = stream.str();
		}

		/**
		 * Constructor of the modbus exception
		 * Adds a user specified message
		 * @see ModbusException
		 **/
        ModbusException(const std::string msg) : std::runtime_error(""), errorCode(errno), message(""){
			std::stringstream stream;
			stream << msg << std::endl;
			stream << "modbus error[" << errorCode << "]: " << modbus_strerror(errorCode);
			message = stream.str();
		}

		/**
		 * Deconstructor
		 * Use for modbus error extends
		 **/
		virtual ~ModbusException(void) throw(){}

		/**
		 * what getter
		 * @return const char* The error message
		 **/
		virtual const char* what(void) const throw(){
			return message.c_str();
		}

		/**
		 * Error code getter
		 * @return int The modbus error code
		 **/
		int getErrorCode(void){
			return errorCode;
		}
	};
}
