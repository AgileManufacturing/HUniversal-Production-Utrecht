//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        modbus_ctrl.cpp
// File:           wrapper for libmodbus with some extra functionality
// Description:    Lukas Vermond & Kasper van Nieuwland
// Author:         -
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


#include <huniplacer/modbus_ctrl.h>
#include <huniplacer/modbus_exception.h>
#include <huniplacer/utils.h>

#include <sstream>
#include <string>
#include <stdexcept>
#include <boost/thread.hpp>
#include <huniplacer/CRD514_KD.h>
#include <cstdio>
#include <iostream>

/**
 * modbus_ctrl.cpp -> Class that implements the modbus protocol
 *
 * Modbus communication is realised with use of libmodbus.
 * This class merely adds a couple of features:
 * - timing control
 * - shadowing of certain registers
 * - the ability to write 32-bit values (instead of only 16-bit values)
 * - thread safety
 **/

namespace huniplacer
{
    /**
     * Constructor
     * @param context initialized modbus_t
     **/
    modbus_ctrl::modbus_ctrl(modbus_t* context) :
        next_write_time(0),
        shadow_registers()
    {
        this->context = context;
        if(context == NULL)
        {
            throw modbus_exception();
        }
        
        //set timeout
        struct timeval timeout_end;
        struct timeval timeout_begin;
        modbus_get_byte_timeout(context, &timeout_end);
        timeout_end.tv_usec = TIMEOUT_END;
        modbus_set_byte_timeout(context, &timeout_end);

        modbus_get_response_timeout(context, &timeout_begin);
        timeout_begin.tv_usec = TIMEOUT_BEGIN;
        modbus_set_response_timeout(context, &timeout_begin);
        
        //connect
        if(modbus_connect(context) == -1)
        {
            throw modbus_exception();
        }
    }

    modbus_ctrl::~modbus_ctrl(void)
    {
        modbus_close(context);
        modbus_free(context);
    }

    /**
     * Utility function. used to wait the remaining time till next_write_time.
     **/
    void modbus_ctrl::wait(void)
    {
        long delta = next_write_time - utils::time_now();
        if(delta > 0)
        {
            utils::sleep(delta);
        }
    }

    /**
     * Calculate a 64-bit value representing the slave and register address
     **/
    uint64_t modbus_ctrl::get_shadow_address(crd514_kd::slaves::t slave, uint16_t address)
    {
    	uint16_t s = (uint16_t)slave;
    	return (s << 16) | address;
    }

    /**
     * Used to access 16-bit shadow registers
     * @param slave slave address
     * @param address the register address
     * @param out_value output parameter, the value gets stored here
     * @return true if the value was shadowed, false otherwise
     **/
    bool modbus_ctrl::get_shadow(crd514_kd::slaves::t slave, uint32_t address, uint16_t& out_value)
    {
        uint64_t a = get_shadow_address(slave, address);
        shadow_map::iterator it = shadow_registers.find(a);
        if(it == shadow_registers.end())
        {
            return false;
        }
        out_value = it->second;
        return true;
    }
    
    /**
     * Used to write 16-bit values to the shadow registers
     * @param slave slave address
     * @param address the registers address
     * @param value the value that will be written
     **/
    void modbus_ctrl::set_shadow(crd514_kd::slaves::t slave, uint32_t address, uint16_t value)
    {
        shadow_registers[get_shadow_address(slave, address)] = value;
    }

    /**
     * Used to write 32-bit values to the shadow registers
     * @param slave slave address
     * @param address the registers address
     * @param value the value that will be written
     **/    
    void modbus_ctrl::set_shadow32(crd514_kd::slaves::t slave, uint32_t address, uint32_t value)
    {
        shadow_registers[get_shadow_address(slave, address+0)] = (value >> 16) & 0xFFFF;
        shadow_registers[get_shadow_address(slave, address+1)] = value & 0xFFFF;
    }

    /**
     * Write a 16-bit value over modbus
     * @param slave the slave address
     * @param address the register address
     * @param data data that will be written
     * @param use_shadow will check if write is necesary by first checking the shadow registers if true
     **/
    void modbus_ctrl::write_u16(crd514_kd::slaves::t slave, uint16_t address, uint16_t data, bool use_shadow)
    {
        if(use_shadow)
        {
            uint16_t shadow_data;
            if(get_shadow(slave, address, shadow_data) && shadow_data == data)
            {
                return;
            }
        }
        
        wait();
        modbus_set_slave(context, slave);
        int r = modbus_write_register(context, (int)address, (int)data);
        next_write_time = utils::time_now() + (slave == crd514_kd::slaves::BROADCAST ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            //when broadcasting; ignore timeout errors
            if(slave == crd514_kd::slaves::BROADCAST && errno == MODBUS_ERRNO_TIMEOUT)
            {
                return;
            }
            
            throw modbus_exception();

        }
        
        if(use_shadow)
        {
        	set_shadow(slave, address, data);
        }
    }

    /**
     * Write an array 16-bit values over modbus
     * @param slave the slave address
     * @param first_address the first registers address
     * @param data data that will be written
     * @param len data length (in words)
     **/
    void modbus_ctrl::write_u16(crd514_kd::slaves::t slave, uint16_t first_address, uint16_t* data, unsigned int len)
    {
        if(len > 10)
        {
            throw modbus_exception();
        }
        
        wait();
        
        modbus_set_slave(context, slave);
        int r = modbus_write_registers(context, first_address, len, data);
        next_write_time = utils::time_now() + (slave == crd514_kd::slaves::BROADCAST ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            //when broadcasting; ignore timeout errors
            if(slave == crd514_kd::slaves::BROADCAST && errno == MODBUS_ERRNO_TIMEOUT)
            {
                return;
            }
            
            throw modbus_exception();
        }
    }

    /**
     * Write a 32-bit value over modbus
     * @param slave the slave address
     * @param address the register address
     * @param data data that will be written
     * @param use_shadow will check if write is necesary by first checking the shadow registers if true
     **/    
    void modbus_ctrl::write_u32(crd514_kd::slaves::t slave, uint16_t address, uint32_t data, bool use_shadow)
    {
    	try
    	{
			uint16_t _data[2];
			_data[0] = (data >> 16) & 0xFFFF;
			_data[1] = data & 0xFFFF;

			if(use_shadow)
			{
				uint16_t shadow_up;
				bool skip_up = get_shadow(slave, address+0, shadow_up) && shadow_up == _data[0];
				uint16_t shadow_lo;
				bool skip_lo = get_shadow(slave, address+1, shadow_lo) && shadow_lo == _data[1];

				if(skip_up && skip_lo)
				{
					return;
				}
				else if(skip_lo) //write only up
				{
					write_u16(slave, address, _data[0]);
					set_shadow(slave, address, _data[0]);
					return;
				}
				else if(skip_up) //write only lo
				{
					write_u16(slave, address+1, _data[1]);
					set_shadow(slave, address+1, _data[1]);
					return;
				}
			}

			//write up & lo
			write_u16(slave, address, _data, 2);

			if(use_shadow)
			{
				set_shadow32(slave, address, data);
			}
    	}
    	catch(modbus_exception& ex)
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
    uint16_t modbus_ctrl::read_u16(crd514_kd::slaves::t slave, uint16_t address)
    {
        wait();
        modbus_set_slave(context, slave);
        uint16_t data;
        int r = modbus_read_registers(context, (int)address, 1, &data); 
        next_write_time = utils::time_now() + (slave == crd514_kd::slaves::BROADCAST ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            throw modbus_exception();
        }
        
        return data;
    }

    /**
     * Read an array of 16-bit values
     * @param slave the slave address
     * @param first_address first registers address from which on data will be read
     * @param data will be stored here
     * @param len data length (in words)
     **/    
    void modbus_ctrl::read_u16(crd514_kd::slaves::t slave, uint16_t first_address, uint16_t* data, unsigned int len)
    {
        wait();
        modbus_set_slave(context, slave);
        int r = modbus_read_registers(context, (int)first_address, len, data); 
        next_write_time = utils::time_now() + (slave == crd514_kd::slaves::BROADCAST ? WRITE_INTERVAL_BROADCAST : WRITE_INTERVAL_UNICAST);
        
        if(r == -1)
        {
            throw modbus_exception();
        }
    }
  
    /**
     * Read a 32-bit value
     * @param slave the slave address
     * @param address address from which will be read
     * @return value that was read
     **/    
    uint32_t modbus_ctrl::read_u32(crd514_kd::slaves::t slave, uint16_t address)
    {
        try
        {
            uint16_t data[2];
            read_u16(slave, address, data, 2);
            return ((data[0] << 16) & 0xFFFF0000) | data[1];
        }
        catch(modbus_exception& ex)
        {
            throw ex;
        }

        return 0; //to suppress warning
    }
}
