//******************************************************************************
//
//                 REXOS
//
//******************************************************************************
// Project:        ModbusController.cpp
// File:           wrapper for libmodbus with some extra functionality
// Description:    
// Author:      1.0 Lukas Vermond & Kasper van Nieuwland
//              1.1 Koen Braham
// Notes:          
//
// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************


#include <ModbusController/ModbusController.h>
#include <ModbusController/ModbusException.h>
#include <Utilities/Utilities.h>

#include <sstream>
#include <string>
#include <stdexcept>
#include <boost/thread.hpp>
#include <cstdio>
#include <iostream>

/**
 * ModbusController.cpp -> Class that implements the modbus protocol
 *
 * Modbus communication is realised with use of libmodbus.
 * This class merely adds a couple of features:
 * - timing control
 * - shadowing of certain registers
 * - the ability to write 32-bit values (instead of only 16-bit values)
 * - thread safety
 **/

namespace ModbusController
{
    /**
     * Constructor
     * @param context initialized modbus_t
     **/
    ModbusController::ModbusController(modbus_t* context) :
        nextWriteTime(0),
        shadowRegisters()
    {
        this->context = context;
        if(context == NULL)
        {
            throw ModbusException("Error uninitialized connection");
        }
        
        //set timeout
        struct timeval timeoutEnd;
        struct timeval timeoutBegin;
        modbus_get_byte_timeout(context, &timeoutEnd);
        timeoutEnd.tv_usec = TIMEOUT_END;
        modbus_set_byte_timeout(context, &timeoutEnd);

        modbus_get_response_timeout(context, &timeoutBegin);
        timeoutBegin.tv_usec = TIMEOUT_BEGIN;
        modbus_set_response_timeout(context, &timeoutBegin);
        
        #ifdef MODBUS_LOGGING
            logFile.open(MODBUS_LOGGING);
            if(!logFile.is_open()){
                throw ModbusException( "File Error!");
            }
            logFile << "Start logging " << std::endl;
        #endif

        //connect
        if(modbus_connect(context) == -1)
        {
            throw ModbusException("Unable to connect modbus");
        }
    }

    ModbusController::~ModbusController(void)
    {
        #ifdef MODBUS_LOGGING
            logFile.close();
        #endif
        modbus_close(context);
        modbus_free(context);
    }

    /**
     * Utility function. used to wait the remaining time till nextWriteTime.
     **/
    void ModbusController::wait(void)
    {
        long delta = nextWriteTime - Utilities::timeNow();
        if(delta > 0)
        {
            Utilities::sleep(delta);
        }
    }

    /**
     * Calculate a 64-bit value representing the crd514-kd motorcontroller and register address
     * @param slave crd514-kd motorcontroller address
     * @param address the register address
     * @return the 64-bit motorcontroller and register address value
     **/
    uint64_t ModbusController::getShadowAddress(uint16_t slave, uint16_t address)
    {
    	return (slave << 16) | address;
    }

    /**
     * Reads a 16-bit shadow register
     * @param slave crd514-kd motorcontroller address
     * @param address the register address
     * @param outValue output parameter, the value gets stored here
     * @return true if the value was shadowed, false otherwise
     **/
    bool ModbusController::getShadow(uint16_t slave, uint32_t address, uint16_t& outValue)
    {
        uint64_t a = getShadowAddress(slave, address);
        ShadowMap::iterator it = shadowRegisters.find(a);
        if(it == shadowRegisters.end())
        {
            return false;
        }
        outValue = it->second;
        return true;
    }
    
    /**
     * Writes a 16-bit value to a shadow register
     * @param slave crd514-kd motorcontroller address
     * @param address the register's address
     * @param value the value that will be written
     **/
    void ModbusController::setShadow(uint16_t slave, uint32_t address, uint16_t value)
    {
        shadowRegisters[getShadowAddress(slave, address)] = value;
    }

    /**
     * Writes a 32-bit value to a shadow register
     * @param slave crd514-kd motorcontroller address
     * @param address the register's address
     * @param value the value that will be written
     **/    
    void ModbusController::setShadow32(uint16_t slave, uint32_t address, uint32_t value)
    {
        shadowRegisters[getShadowAddress(slave, address+0)] = (value >> 16) & 0xFFFF;
        shadowRegisters[getShadowAddress(slave, address+1)] = value & 0xFFFF;
    }

    /**
     * Write a 16-bit value over modbus
     * @param slave crd514-kd motorcontroller address
     * @param address the register address
     * @param data data that will be written
     * @param use_shadow will check if write is necesary by first checking the shadow registers if true
     **/
    void ModbusController::writeU16(uint16_t slave, uint16_t address, uint16_t data, bool useShadow)
    {
        #ifdef MODBUS_LOGGING
            logFile << "WriteU16\t" << slave << "\t" << address << "\t" << data << std::endl;
        #endif
        if(useShadow)
        {
            uint16_t shadowData;
            if(getShadow(slave, address, shadowData) && shadowData == data)
            {
                return;
            }
        }
        
        wait();
        modbus_set_slave(context, slave);
        int r = modbus_write_register(context, (int)address, (int)data);
        // TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
        nextWriteTime = Utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            //when broadcasting; ignore timeout errors
            if(slave == 0 && errno == MODBUS_ERRNO_TIMEOUT)
            {
                return;
            }
            
            throw ModbusException("Error writing u16");

        }
        
        if(useShadow)
        {
        	setShadow(slave, address, data);
        }
    }

    /**
     * Write an array of 16-bit values over modbus
     * @param slave crd514-kd motorcontroller address
     * @param first_address the first register's address
     * @param data data that will be written
     * @param length data length (in words)
     **/
    void ModbusController::writeU16(uint16_t slave, uint16_t first_address, uint16_t* data, unsigned int length)
    {
        if(length > 10)
        {
            throw ModbusException("length > 10");
        }
        
        wait();
        
        #ifdef MODBUS_LOGGING
            for(unsigned int i = 0; i < length; i++){
                logFile << "WriteU16Array\t" << slave << "\t" << (first_address + i) << "\t" << data[i] << std::endl;
            }
        #endif
        

        modbus_set_slave(context, slave);
        int r = modbus_write_registers(context, first_address, length, data);
        // TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
        nextWriteTime = Utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            //when broadcasting; ignore timeout errors
            if(slave == 0 && errno == MODBUS_ERRNO_TIMEOUT)
            {
                return;
            }
            
            throw ModbusException("Error writing u16 array");
        }
    }

    /**
     * Write a 32-bit value over modbus
     * @param slave crd514-kd motorcontroller address
     * @param address the register's address
     * @param data data that will be written
     * @param use_shadow will check if write is necesary by first checking the shadow registers if true
     **/    
    void ModbusController::writeU32(uint16_t slave, uint16_t address, uint32_t data, bool useShadow)
    {
        #ifdef MODBUS_LOGGING
            logFile << "WriteU32\t" << slave << "\t" << address << "\t" << data << std::endl;
        #endif
    	try
    	{
			uint16_t _data[2];
			_data[0] = (data >> 16) & 0xFFFF;
			_data[1] = data & 0xFFFF;

			if(useShadow)
			{
				uint16_t shadowHigh;
                uint16_t shadowLow;
				bool skipHigh = getShadow(slave, address+0, shadowHigh) && shadowHigh == _data[0];
				bool skipLow = getShadow(slave, address+1, shadowLow) && shadowLow == _data[1];

				if(skipHigh && skipLow)
				{
					return;
				}
				else if(skipLow) //write only up
				{
					writeU16(slave, address, _data[0]);
					setShadow(slave, address, _data[0]);
					return;
				}
				else if(skipHigh) //write only lo
				{
					writeU16(slave, address+1, _data[1]);
					setShadow(slave, address+1, _data[1]);
					return;
				}
			}

			//write up & lo
			writeU16(slave, address, _data, 2);

			if(useShadow)
			{
				setShadow32(slave, address, data);
			}
    	}
    	catch(ModbusException& ex)
    	{
    		throw ex;
    	}
    }

    /**
     * Read a 16-bit value
     * @param slave slave address
     * @param address address that will be read from
     * @return the value that was read
     **/    
    uint16_t ModbusController::readU16(uint16_t slave, uint16_t address)
    {
        wait();
        modbus_set_slave(context, slave);
        uint16_t data;
        int r = modbus_read_registers(context, (int)address, 1, &data); 
        // TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
        nextWriteTime = Utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            throw ModbusException("Error reading u16");
        }
        
        #ifdef MODBUS_LOGGING
            logFile << "ReadU16\t" << slave << "\t" << address << "\t" << data << std::endl;
        #endif
        return data;
    }

    /**
     * Read an array of 16-bit values
     * @param slave the slave address
     * @param first_address first registers address from which on data will be read
     * @param data will be stored here
     * @param length data length (in words)
     **/    
    void ModbusController::readU16(uint16_t slave, uint16_t first_address, uint16_t* data, unsigned int length)
    {
        wait();
        modbus_set_slave(context, slave);
        int r = modbus_read_registers(context, (int)first_address, length, data); 
        // TODO: fix the broadcast issue slave == crd514_kd::slaves::BROADCAST temporary == 0
        nextWriteTime = Utilities::timeNow() + (slave == 0 ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            throw ModbusException("Error reading u16 array");
        }

        #ifdef MODBUS_LOGGING
            for(unsigned int i = 0; i < length; i++){
                logFile << "ReadU16Array\t" << slave << "\t" << (first_address + i) << "\t" << data[i] << std::endl;
            }
        #endif
    }
  
    /**
     * Read a 32-bit value
     * @param slave the slave address
     * @param address address from which will be read
     * @return value that was read
     **/    
    uint32_t ModbusController::readU32(uint16_t slave, uint16_t address)
    {
        try
        {
            uint16_t data[2];
            readU16(slave, address, data, 2);

            #ifdef MODBUS_LOGGING
                logFile << "ReadU32\t" << slave << "\t" << address << "\t" << (((data[0] << 16) & 0xFFFF0000) | data[1]) << std::endl;
            #endif
            return ((data[0] << 16) & 0xFFFF0000) | data[1];
        }
        catch(ModbusException& ex)
        {
            throw ex;
        }

        return 0; //to suppress warning
    }
}
