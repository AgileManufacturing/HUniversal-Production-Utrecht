//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           modbus_ctrl.h
// Description:    wrapper for libmodbus with some extra functionality
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          -
//
// License:        GNU GPL v3
//
// This file is part of huniplacer.
//
// huniplacer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// huniplacer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with huniplacer.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#pragma once

#include "huniplacer/CRD514_KD.h"

extern "C"
{
    #include <modbus/modbus.h>
}

#include <stdint.h>
#include <string>
#include <boost/thread.hpp>
#include <map>

namespace huniplacer
{
    /**
     * @brief class that implements the modbus protocol
     *
     * modbus communication is realised with use of libmodbus.
     * this class merely adds a couple of features:
     * - timing control
     * - shadowing of certain registers
     * - the ability to write 32-bit values (instead of only 16-bit values)
     * - thread safety
     **/
    class modbus_ctrl
    {
        private:
            enum
            {
                MODBUS_ERRNO_TIMEOUT = 0x6E,
                
                WRITE_INTERVAL_UNICAST   = 8,  //ms
                WRITE_INTERVAL_BROADCAST = 16, //ms
                
                TIMEOUT_BEGIN = 150000, //us
                TIMEOUT_END   = 150000, //us
            };
            
            modbus_t* context;
            long next_write_time;
            
            typedef std::map<uint64_t, uint16_t> shadow_map;

            /// @brief values at certain addresses are shadowed here
            shadow_map shadow_registers;

            /**
             * @brief utility function. used to wait the remaining time till next_write_time
             **/
            void wait(void);

            /**
             * @brief calculate a 64-bit value representing the slave and register address
             **/
            uint64_t get_shadow_address(crd514_kd::slaves::t slave, uint16_t address);

            /**
             * @brief used to access 16-bit shadow registers
             * @param slave slave address
             * @param address the register address
             * @param out_value output parameter, the value gets stored here
             * @return true if the value was shadowed, false otherwise
             **/
            bool get_shadow(crd514_kd::slaves::t slave, uint32_t address, uint16_t& out_value);

            /**
             * @brief used to write 16-bit values to the shadow registers
             * @param slave slave address
             * @param address the registers address
             * @param value the value that will be written
             **/
            void set_shadow(crd514_kd::slaves::t slave, uint32_t address, uint16_t value);

            /**
			 * @brief used to write 32-bit values to the shadow registers
			 * @param slave slave address
			 * @param address the registers address
			 * @param value the value that will be written
			 **/
            void set_shadow32(crd514_kd::slaves::t slave, uint32_t address, uint32_t value);
            
        public:
            /**
             * @brief constructor
             * @param context initialized modbus_t
             **/
            modbus_ctrl(modbus_t* context);

            ~modbus_ctrl(void);

            /**
             * @brief write a 16-bit value over modbus
             * @param slave the slave address
             * @param address the register address
             * @param data data that will be written
             * @param use_shadow will check if write is necesary by first checking the shadow registers if true
             **/
            void write_u16(crd514_kd::slaves::t slave, uint16_t address, uint16_t data, bool use_shadow = false);

            /**
			 * @brief write an array 16-bit values over modbus
			 * @param slave the slave address
			 * @param first_address the first registers address
			 * @param data data that will be written
			 * @param len data length (in words)
			 **/
            void write_u16(crd514_kd::slaves::t slave, uint16_t first_address, uint16_t* data, unsigned int len);

            /**
			 * @brief write a 32-bit value over modbus
			 * @param slave the slave address
			 * @param address the register address
			 * @param data data that will be written
			 * @param use_shadow will check if write is necesary by first checking the shadow registers if true
			 **/
            void write_u32(crd514_kd::slaves::t slave, uint16_t address, uint32_t data, bool use_shadow = false);

            /**
             * @brief read a 16-bit value
             * @param slave slave address
             * @param address address that will be read from
             * @return the value that was read
             **/
            uint16_t read_u16(crd514_kd::slaves::t slave, uint16_t address);

            /**
             * @brief read an array of 16-bit values
             * @param slave the slave address
             * @param first_address first registers address from which on data will be read
             * @param data will be stored here
             * @param len data length (in words)
             **/
            void read_u16(crd514_kd::slaves::t slave, uint16_t first_address, uint16_t* data, unsigned int len);

            /**
             * @brief read a 32-bit value
             * @param slave the slave address
             * @param address address from which will be read
             * @return value that was read
             **/
            uint32_t read_u32(crd514_kd::slaves::t slave, uint16_t address);
    };
}
