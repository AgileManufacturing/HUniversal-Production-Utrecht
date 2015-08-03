#pragma once

#include <string>
#include <sstream>
#include <cerrno>
#include <modbus/modbus.h>

#include <rexos_io/InputOutputException.h>

namespace rexos_io{
	class ModbusException : public InputOutputException {
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
		ModbusException(void) : InputOutputException(""), errorCode(errno), message(){
			std::stringstream stream;
			stream << "modbus error[" << errorCode << "]: " << modbus_strerror(errorCode);
			message = stream.str();
		}

		/**
		 * Constructor of the modbus exception
		 * Adds a user specified message
		 * @see ModbusException
		 **/
		ModbusException(const std::string msg) : InputOutputException(""), errorCode(errno), message(){
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
