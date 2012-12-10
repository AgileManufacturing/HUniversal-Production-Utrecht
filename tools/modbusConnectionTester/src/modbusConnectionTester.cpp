#include <iostream>
#include <stdint.h>

#include <Motor/CRD514KD.h>
#include <ModbusController/ModbusController.h>
#include <Utilities/Utilities.h>

int main(int argc, char* argv[]) {
	// Open the modbus connection
	//ModbusController::ModbusController* modbus = new ModbusController::ModbusController(modbus_new_tcp("192.168.0.4", 502));
	ModbusController::ModbusController* modbus = new ModbusController::ModbusController(modbus_new_rtu(
	        "/dev/ttyS0",
	        Motor::CRD514KD::RtuConfig::BAUDRATE,
	        Motor::CRD514KD::RtuConfig::PARITY,
	        Motor::CRD514KD::RtuConfig::DATA_BITS,
	        Motor::CRD514KD::RtuConfig::STOP_BITS));

	// Write the motors in the order as they are connected.
	// On write failure of motor 3, check the master device connection!
	// On write failure of motor 2 & 1, check the link cable!
	try {
		for (int i = 3; i > 0; i--) {
			std::cout << "Initializing motor " << i << std::endl;

			// Set common acceleration to 500.000 (1=0.001 ms/kHz)
			modbus->writeU32(i, Motor::CRD514KD::Registers::CFG_COMMON_ACCELERATION, 500000);

			// Set common deceleration to 500.000 (1=0.001 ms/kHz)
			modbus->writeU32(i, Motor::CRD514KD::Registers::CFG_COMMON_DECELERATION, 500000);

			// set step angle to 1/10 (divide by 10)
			modbus->writeU16(i, Motor::CRD514KD::Registers::CFG_MOTOR_STEP_ANGLE, 6);
		}
	} catch (std::exception & ex) {
		std::cerr << "Error in writing to register" << std::endl;
		std::cerr << ex.what() << std::endl;
	}

	delete modbus;

	return 0;
}
