#pragma once

#include <string>

namespace rexos_io {
	/**
	 * Wrapper class for libmodbus with some extra functionality.
	 **/
	class RtuConfig {
	public:
		/**
		 * @var char DEVICE[]
		 * A char array (c string) holding the address of the modbus rtu.
		 **/
		static const char DEVICE[];

		/**
		 * Constants used when constructing a modbus rtu (using modbus_new_rtu).
		 **/
		enum _rtu_config{
			BAUDRATE	= 115200,
			PARITY		= 'E',
			DATA_BITS	= 8,
			STOP_BITS	= 1
		};
	};
	const char RtuConfig::DEVICE[] = "/dev/ttyUSB1";
}
