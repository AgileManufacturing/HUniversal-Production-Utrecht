/**
 * @file ModbusController.cpp
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

#include <rexos_modbus/ModbusController.h>
#include <rexos_modbus/ModbusException.h>
#include <rexos_utilities/Utilities.h>

#include <sstream>
#include <string>
#include <stdexcept>
#include <boost/thread.hpp>
#include <cstdio>
#include <iostream>

namespace rexos_modbus{
	/**
	 * Constructor of a modbuscontroller.
	 * 
	 * @param context Initialized modbus_t.
	 **/
    ModbusController::ModbusController(modbus_t* context) : 
    context(context),
    nextWriteTime(0), 
    shadowRegisters(){
		if(context == NULL){
			throw ModbusException("Error uninitialized connection");
		}

		// Set timeout.
		struct timeval timeoutEnd;
		struct timeval timeoutBegin;
		modbus_get_byte_timeout(context, &timeoutEnd);
		timeoutEnd.tv_usec = TIMEOUT_BYTE;
		modbus_set_byte_timeout(context, &timeoutEnd);

		modbus_get_response_timeout(context, &timeoutBegin);
		timeoutBegin.tv_usec = TIMEOUT_RESPONE;
		modbus_set_response_timeout(context, &timeoutBegin);

		#ifdef MODBUS_LOGGING
			logFile.open(MODBUS_LOGGING);
			if(!logFile.is_open()){
				throw ModbusException("File Error!");
			}
			logFile << "Start logging " << std::endl;
		#endif

		// Connect.
		if(modbus_connect(context) == -1){
			throw ModbusException("Unable to connect modbus");
		}
	}

	/**
	 * Deconstructor of a modbuscontroller, ends logging and closes modbus connection.
	 **/
	ModbusController::~ModbusController(void){
		#ifdef MODBUS_LOGGING
			logFile.close();
		#endif
		modbus_close(context);
		modbus_free(context);
	}

	/**
	* Utility function. used to wait the remaining time till nextWriteTime.
	**/
	void ModbusController::wait(void){
		long delta = nextWriteTime - rexos_utilities::timeNow();
		if(delta > 0){
			rexos_utilities::sleep(delta);
		}
	}

	/**
	 * Calculates a 64-bit value representing the crd514-kd motorcontroller and register address.
	 * 
	 * @param slave crd514-kd motorcontroller address.
	 * @param address The register address.
	 * 
	 * @return the 64-bit motorcontroller and register address value.
	 **/
	uint64_t ModbusController::getShadowAddress(uint16_t slave, uint16_t address){
		return (slave << 16) | address;
	}

	/**
	 * Reads a 16-bit shadow register.
	 * 
	 * @param slave crd514-kd motorcontroller address.
	 * @param address The register address.
	 * @param outValue Output parameter, the value gets stored here.
	 * 
	 * @return true if the value was shadowed, false otherwise.
	 **/
	bool ModbusController::getShadow(uint16_t slave, uint32_t address, uint16_t& outValue){
		uint64_t shadowAddress = getShadowAddress(slave, address);
		ShadowMap::iterator it = shadowRegisters.find(shadowAddress);
		if(it == shadowRegisters.end()){
			return false;
		}
		outValue = it->second;
		return true;
	}

	/**
	 * Writes a 16-bit value to a shadow register.
	 * 
	 * @param slave crd514-kd motorcontroller address.
	 * @param address The register's address.
	 * @param value The value that will be written.
	 **/
	void ModbusController::setShadow(uint16_t slave, uint32_t address, uint16_t value){
		shadowRegisters[getShadowAddress(slave, address)] = value;
	}

	/**
	 * Writes a 32-bit value to a shadow register.
	 * 
	 * @param slave crd514-kd motorcontroller address.
	 * @param address The register's address.
	 * @param value The value that will be written.
	 **/
	void ModbusController::setShadow32(uint16_t slave, uint32_t address, uint32_t value){
		shadowRegisters[getShadowAddress(slave, address+0)] = (value >> 16) & 0xFFFF;
		shadowRegisters[getShadowAddress(slave, address+1)] = value & 0xFFFF;
	}

	/**
	 * Write a 16-bit value over modbus.
	 * 
	 * @param slave crd514-kd motorcontroller address.
	 * @param address The register address.
	 * @param data Data that will be written.
	 * @param useShadow If true is passed, it will check if writing is necessary by first checking the shadow registers.
	 **/
	void ModbusController::writeU16(uint16_t slave, uint16_t address, uint16_t data, bool useShadow){
		#ifdef MODBUS_LOGGING
			logFile << "WriteU16\t" << slave << "\t" << address << "\t" << data << std::endl;
		#endif
		if(useShadow){
			uint16_t shadowData;
			if(getShadow(slave, address, shadowData) && shadowData == data){
				return;
			}
		}

		wait();
		modbus_set_slave(context, slave);
		int r = modbus_write_register(context, (int)address, (int)data);

		// TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
		nextWriteTime = rexos_utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);

		if(r == -1){
			// When broadcasting; ignore timeout errors.
			if(slave == 0 && errno == MODBUS_ERRNO_TIMEOUT){
				return;
			}
			
			throw ModbusException("Error writing u16");
		}

		if(useShadow){
			setShadow(slave, address, data);
		}
	}

	/**
	 * Write an array of 16-bit values over modbus.
	 * 
	 * @param slave crd514-kd motorcontroller address.
	 * @param firstAddress The first register's address.
	 * @param data Data that will be written.
	 * @param length Data length (in words).
	 **/
	void ModbusController::writeU16(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length){
		if(length > 10){
			throw ModbusException("length > 10");
		}

		wait();

		#ifdef MODBUS_LOGGING
			for(unsigned int i = 0; i < length; i++){
				logFile << "WriteU16Array\t" << slave << "\t" << (firstAddress + i) << "\t" << data[i] << std::endl;
			}
		#endif

		modbus_set_slave(context, slave);
		int r = modbus_write_registers(context, firstAddress, length, data);

		// TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
		nextWriteTime = rexos_utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);

		if(r == -1){
			// When broadcasting; ignore timeout errors
			if(slave == 0 && errno == MODBUS_ERRNO_TIMEOUT){
				return;
			}

			throw ModbusException("Error writing u16 array");
		}
	}

	/**
	 * Write a 32-bit value over modbus.
	 * 
	 * @param slave crd514-kd motorcontroller address.
	 * @param address The register's address.
	 * @param data Data that will be written.
	 * @param useShadow If true is passed, it will check if writing is necessary by first checking the shadow registers.
	 **/
	void ModbusController::writeU32(uint16_t slave, uint16_t address, uint32_t data, bool useShadow){
		#ifdef MODBUS_LOGGING
			logFile << "WriteU32\t" << slave << "\t" << address << "\t" << data << std::endl;
		#endif
		try{
			uint16_t _data[2];
			_data[0] = (data >> 16) & 0xFFFF;
			_data[1] = data & 0xFFFF;

			if(useShadow){
				uint16_t shadowHigh;
				uint16_t shadowLow;
				bool skipHigh = getShadow(slave, address+0, shadowHigh) && shadowHigh == _data[0];
				bool skipLow = getShadow(slave, address+1, shadowLow) && shadowLow == _data[1];

				if(skipHigh && skipLow){
					return;
				} else if(skipLow){
					// Write high only
					writeU16(slave, address, _data[0]);
					setShadow(slave, address, _data[0]);
					return;
				} else if(skipHigh){
					// Write low only
					writeU16(slave, address+1, _data[1]);
					setShadow(slave, address+1, _data[1]);
					return;
				}
			}

			// Write high & low
			writeU16(slave, address, _data, 2);

			if(useShadow){
				setShadow32(slave, address, data);
			}
		} catch(ModbusException& exception){
			throw exception;
		}
	}

	/**
	 * Read a 16-bit value.
	 * 
	 * @param slave Slave address.
	 * @param address Address that will be read from.
	 * 
	 * @return the value that was read.
	 **/
	uint16_t ModbusController::readU16(uint16_t slave, uint16_t address){
		wait();
		modbus_set_slave(context, slave);
		uint16_t data;
		int r = modbus_read_registers(context, (int)address, 1, &data);

		// TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
		nextWriteTime = rexos_utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);

		if(r == -1){
			throw ModbusException("Error reading u16");
		}

		#ifdef MODBUS_LOGGING
			logFile << "ReadU16\t" << slave << "\t" << address << "\t" << data << std::endl;
		#endif
		return data;
	}

	/**
	 * Read an array of 16-bit values.
	 * 
	 * @param slave The slave address.
	 * @param firstAddress First registers address from which on data will be read.
	 * @param data Will be stored here.
	 * @param length Data length (in words).
	 **/
	void ModbusController::readU16(uint16_t slave, uint16_t firstAddress, uint16_t* data, unsigned int length){
		wait();
		modbus_set_slave(context, slave);
		int r = modbus_read_registers(context, (int)firstAddress, length, data);

		// TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
		nextWriteTime = rexos_utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);

		if(r == -1){
			throw ModbusException("Error reading u16 array");
		}

		#ifdef MODBUS_LOGGING
			for(unsigned int i = 0; i < length; i++){
				logFile << "ReadU16Array\t" << slave << "\t" << (firstAddress + i) << "\t" << data[i] << std::endl;
			}
		#endif
	}

	/**
	 * Read a 32-bit value.
	 * 
	 * @param slave The slave address.
	 * @param address Address from which will be read.
	 * 
	 * @return value that was read.
	 **/	
	uint32_t ModbusController::readU32(uint16_t slave, uint16_t address){
		try{
			uint16_t data[2];
			readU16(slave, address, data, 2);

			#ifdef MODBUS_LOGGING
				logFile << "ReadU32\t" << slave << "\t" << address << "\t" << (((data[0] << 16) & 0xFFFF0000) | data[1]) << std::endl;
			#endif
			return ((data[0] << 16) & 0xFFFF0000) | data[1];
		} catch(ModbusException& exception){
			throw exception;
		}

		// TODO: Why does this suppress a warning, what warning and why is it solved like this?
		// To suppress warning.
		return 0;
	}
}
