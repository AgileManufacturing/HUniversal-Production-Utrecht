#include <rexos_io/ModbusInputOutputController.h>
#include <rexos_io/ModbusException.h>
#include <rexos_io/ShadowException.h>
#include <rexos_utilities/Utilities.h>

namespace rexos_io {
	ModbusInputOutputController::ModbusInputOutputController() :
			context(NULL), nextWriteTime(0) {
#ifdef MODBUS_LOGGING
		logFile.open(MODBUS_LOGGING);
		if(!logFile.is_open()) {
			throw ModbusException("File Error!");
		}
		logFile << "Start logging " << std::endl;
#endif
	}
	ModbusInputOutputController::~ModbusInputOutputController() {
#ifdef MODBUS_LOGGING
		logFile.close();
#endif
		if(context != NULL) {
			modbus_close(context);
		}
	}
	
	void ModbusInputOutputController::initializeModbus(modbus_t* context) {
		if(context == NULL){
			throw ModbusException("Error uninitialized connection");
		}
		// Set timeout.
		struct timeval timeoutEnd;
		modbus_get_byte_timeout(context, &timeoutEnd);
		timeoutEnd.tv_usec = TIMEOUT_BYTE;
		modbus_set_byte_timeout(context, &timeoutEnd);

		struct timeval timeoutBegin;
		modbus_get_response_timeout(context, &timeoutBegin);
		timeoutBegin.tv_usec = TIMEOUT_RESPONE;
		modbus_set_response_timeout(context, &timeoutBegin);


		// Connect.
		if(modbus_connect(context) == -1) {
			throw ModbusException("Unable to connect modbus");
		}
		this->context = context;
	}
	
	void ModbusInputOutputController::setCurrentSlave(uint16_t slave) {
		modbus_set_slave(context, slave);
		currentSlave = slave;
	}
	
	void ModbusInputOutputController::writeU16(uint16_t firstAddress, uint16_t* data, unsigned int length){
		if(length > 10){
			throw ModbusException("length > 10");
		}

		wait();

#ifdef MODBUS_LOGGING
		for(unsigned int i = 0; i < length; i++){
			logFile << "WriteU16Array\t" << curr << "\t" << (firstAddress + i) << "\t" << data[i] << std::endl;
		}
#endif
		
		int r = modbus_write_registers(context, firstAddress, length, data);
		
		nextWriteTime = rexos_utilities::timeNow() + (currentSlave == MODBUS_BROADCAST_ADDRESS ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);

		if(r == -1){
			// When broadcasting; ignore timeout errors
			if(currentSlave == MODBUS_BROADCAST_ADDRESS && errno == MODBUS_ERRNO_TIMEOUT) return;
			else throw ModbusException("Error writing u16 array");
		}
	}
	
	void ModbusInputOutputController::readU16(uint16_t firstAddress, uint16_t* data, unsigned int length) {
		wait();
		int r = modbus_read_registers(context, (int)firstAddress, length, data);

		nextWriteTime = rexos_utilities::timeNow() + (currentSlave == MODBUS_BROADCAST_ADDRESS ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);

		if(r == -1){
			throw ModbusException("Error reading u16 array");
		}

#ifdef MODBUS_LOGGING
		for(unsigned int i = 0; i < length; i++){
			logFile << "ReadU16Array\t" << currentSlave << "\t" << (firstAddress + i) << "\t" << data[i] << std::endl;
		}
#endif
	}
	
	
	void ModbusInputOutputController::writeShadowU16(uint16_t address, uint16_t value) {
		// shadowRegistry entry will be created if slave did not exist yet
		InputOutputControllerInterface::writeShadowU16(address, value, shadowRegistry[currentSlave]);
	}
	void ModbusInputOutputController::writeShadowU32(uint16_t address, uint32_t value) {
		// shadowRegistry entry will be created if slave did not exist yet
		InputOutputControllerInterface::writeShadowU32(address, value, shadowRegistry[currentSlave]);
	}
	uint16_t ModbusInputOutputController::readShadowU16(uint16_t address) {
		// shadowRegistry entry will be created if slave did not exist yet
		return InputOutputControllerInterface::readShadowU16(address, shadowRegistry[currentSlave]);
	}
	uint32_t ModbusInputOutputController::readShadowU32(uint16_t address) {
		// shadowRegistry entry will be created if slave did not exist yet
		return InputOutputControllerInterface::readShadowU32(address, shadowRegistry[currentSlave]);
	}
	
	void ModbusInputOutputController::wait(void){
		long delta = nextWriteTime - rexos_utilities::timeNow();
		if(delta > 0){
			rexos_utilities::sleep(delta);
		}
	}
}