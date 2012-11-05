#include <iostream>
#include <InputOutput/OutputDevices/Gripper.h>

void error( ) {
	std::cout << "Gripper handler warning!" << std::endl;
}

/**
 * The IP of the modbus we are connecting to
 **/
#define MODBUS_IP "192.168.0.2"
/**
 * The port we are connecting to
 **/
#define MODBUS_PORT 502

/**
 *
 */
int main(void) {
	// Initialize modbus for IO controller
	std::cout << "[DEBUG] Opening modbus connection" << std::endl;
	modbus_t* modbusIO = modbus_new_tcp(MODBUS_IP, MODBUS_PORT);
	if (modbusIO == NULL) {
		throw std::runtime_error("Unable to allocate libmodbus context");
	}
	if (modbus_connect(modbusIO) == -1) {
		throw std::runtime_error("Modbus connection to IO controller failed");
	}
	assert(modbusIO != NULL);

	ModbusController::ModbusController modbus(modbusIO);

	std::cout << "[DEBUG] Opening IO Controller" << std::endl;
	InputOutput::InputOutputController controller(modbus);

	std::cout << "[DEBUG] Starting gripper" << std::endl;
	InputOutput::OutputDevices::Gripper grp(controller, error);

	std::cout << "Welcome to the gripper watchdog tester!" << std::endl;
	std::cout << "Keys to control the gripper" << std::endl;
	std::cout << "0\tTurn gripper off" << std::endl;
	std::cout << "1\tTurn gripper on" << std::endl;
	std::cout << "-1\t Exit program" << std::endl;

	int input;
	do {
		std::cin >> input;
		if (input == 1) {
			grp.grab();
		} else if (input == 0) {
			grp.release();
		}
	} while (input != -1);

	return 0;
}
