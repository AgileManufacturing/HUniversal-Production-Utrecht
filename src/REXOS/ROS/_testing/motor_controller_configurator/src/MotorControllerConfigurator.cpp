#include <iostream>

#include <rexos_utilities/Utilities.h>
#include <rexos_io/RtuModbusInputOutputController.h>
#include "ros/ros.h"
#include <rexos_motor/CRD514KD.h>
#include "rexos_logger/rexos_logger.h"



int main(int argc, char **argv){
	ros::init(argc, argv, "motor_controller_configurator");
	ros::start();
	ros::Rate loop_rate(15);
	
	while(ros::ok()) {
		rexos_io::RtuModbusInputOutputController modbus = rexos_io::RtuModbusInputOutputController();
		
		REXOS_INFO("Device number to configure: ");
		std::string deviceNumberString;
		std::cin >> deviceNumberString;
		uint16_t deviceNumber = rexos_utilities::stringToInt(deviceNumberString);
		
		REXOS_INFO("Address to use: ");
		std::string addressString;
		std::cin >> addressString;
		uint16_t address = rexos_utilities::stringToInt(addressString);
		
		REXOS_INFO("field length (16 or 32 bits): ");
		std::string fieldLengthString;
		std::cin >> fieldLengthString;
		uint16_t fieldLength = rexos_utilities::stringToInt(fieldLengthString);
		
		REXOS_INFO_STREAM("Value to set (leave empty for read): ");
		std::string valueString;
		std::cin >> valueString;
		if(valueString.length() == 0) {
			// reading mode
			if(fieldLength == 16) {
				REXOS_INFO_STREAM("@" << address + ": " << modbus.readU16(deviceNumber, address));
			} else if(fieldLength == 32) {
				REXOS_INFO_STREAM("@" << address + ": " << modbus.readU32(deviceNumber, address));
				
			}
		} else {
				// writing mode
			uint32_t value = rexos_utilities::stringToInt(valueString);
			if(fieldLength == 16) {
				modbus.writeU16(deviceNumber, address, value, false);
				REXOS_INFO_STREAM("@" << address + "= " << modbus.readU16(deviceNumber, address));
			} else if(fieldLength == 32) {
				modbus.writeU32(deviceNumber, address, value, false);
				REXOS_INFO_STREAM("@" << address + "= " << modbus.readU32(deviceNumber, address));
			}
			
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}
