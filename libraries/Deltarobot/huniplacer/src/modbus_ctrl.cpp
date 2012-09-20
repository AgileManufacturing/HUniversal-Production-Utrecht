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
// License:        GNU GPL v3
//
// This file is part of modbus_ctrl.cpp.
//
// modbus_ctrl.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// modbus_ctrl.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with modbus_ctrl.cpp.  If not, see <http://www.gnu.org/licenses/>.
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

namespace huniplacer
{
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

    void modbus_ctrl::wait(void)
    {
        long delta = next_write_time - utils::time_now();
        if(delta > 0)
        {
            utils::sleep(delta);
        }
    }

    uint64_t modbus_ctrl::get_shadow_address(crd514_kd::slaves::t slave, uint16_t address)
    {
    	uint16_t s = (uint16_t)slave;
    	return (s << 16) | address;
    }

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
    
    void modbus_ctrl::set_shadow(crd514_kd::slaves::t slave, uint32_t address, uint16_t value)
    {
        shadow_registers[get_shadow_address(slave, address)] = value;
    }
    
    void modbus_ctrl::set_shadow32(crd514_kd::slaves::t slave, uint32_t address, uint32_t value)
    {
        shadow_registers[get_shadow_address(slave, address+0)] = (value >> 16) & 0xFFFF;
        shadow_registers[get_shadow_address(slave, address+1)] = value & 0xFFFF;
    }

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
