#include <rexos_io/RtuModbusInputOutputController.h>
#include <rexos_io/ModbusException.h>
#include <rexos_io/RtuConfig.h>
#include <rexos_utilities/Utilities.h>

namespace rexos_io {
	RtuModbusInputOutputController::RtuModbusInputOutputController() {
		modbus_t* context = modbus_new_rtu(
			RtuConfig::DEVICE,
			RtuConfig::BAUDRATE,
			RtuConfig::PARITY,
			RtuConfig::DATA_BITS,
			RtuConfig::STOP_BITS);
		initializeModbus(context);
	}
	RtuModbusInputOutputController::~RtuModbusInputOutputController() {
		modbus_free(context);
	}
	
	void RtuModbusInputOutputController::writeU16(uint16_t slave, uint16_t address, uint16_t value, bool useShadow){
		setCurrentSlave(slave);
		InputOutputControllerInterface::writeU16(address, value, useShadow);
	}
	void RtuModbusInputOutputController::writeU16(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length){
		setCurrentSlave(slave);
		ModbusInputOutputController::writeU16(firstAddress, data, length);
	}
	void RtuModbusInputOutputController::writeU32(uint16_t slave, uint16_t address, uint32_t value, bool useShadow){
		setCurrentSlave(slave);
		InputOutputControllerInterface::writeU32(address, value, useShadow);
	}
	void RtuModbusInputOutputController::readU16(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length) {
		setCurrentSlave(slave);
		ModbusInputOutputController::readU16(firstAddress, data, length);
	}
	uint16_t RtuModbusInputOutputController::readU16(uint16_t slave, uint16_t address, bool useShadow) {
		setCurrentSlave(slave);
		return InputOutputControllerInterface::readU16(address, useShadow);
	}
	uint32_t RtuModbusInputOutputController::readU32(uint16_t slave, uint16_t address, bool useShadow) {
		setCurrentSlave(slave);
		return InputOutputControllerInterface::readU32(address, useShadow);
	}
	
	void RtuModbusInputOutputController::writeShadowU16(uint16_t slave, uint16_t address, uint16_t value) {
		// shadowRegistry entry will be created if slave did not exist yet
		InputOutputControllerInterface::writeShadowU16(address, value, shadowRegistry[slave]);
	}
	void RtuModbusInputOutputController::writeShadowU32(uint16_t slave, uint16_t address, uint32_t value) {
		// shadowRegistry entry will be created if slave did not exist yet
		InputOutputControllerInterface::writeShadowU32(address, value, shadowRegistry[slave]);
	}
	uint16_t RtuModbusInputOutputController::readShadowU16(uint16_t slave, uint16_t address) {
		// shadowRegistry entry will be created if slave did not exist yet
		return InputOutputControllerInterface::readShadowU16(address, shadowRegistry[slave]);
	}
	uint32_t RtuModbusInputOutputController::readShadowU32(uint16_t slave, uint16_t address) {
		// shadowRegistry entry will be created if slave did not exist yet
		return InputOutputControllerInterface::readShadowU32(address, shadowRegistry[slave]);
	}
}