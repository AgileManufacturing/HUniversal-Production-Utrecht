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

            /** 
             * Values at certain addresses are shadowed here
             **/
            shadow_map shadow_registers;

            void wait(void);

            uint64_t get_shadow_address(crd514_kd::slaves::t slave, uint16_t address);
            bool get_shadow(crd514_kd::slaves::t slave, uint32_t address, uint16_t& out_value);
            void set_shadow(crd514_kd::slaves::t slave, uint32_t address, uint16_t value);
            void set_shadow32(crd514_kd::slaves::t slave, uint32_t address, uint32_t value);

        public:
            modbus_ctrl(modbus_t* context);
            ~modbus_ctrl(void);

            void write_u16(crd514_kd::slaves::t slave, uint16_t address, uint16_t data, bool use_shadow = false);
            void write_u16(crd514_kd::slaves::t slave, uint16_t first_address, uint16_t* data, unsigned int len);
            void write_u32(crd514_kd::slaves::t slave, uint16_t address, uint32_t data, bool use_shadow = false);
            uint16_t read_u16(crd514_kd::slaves::t slave, uint16_t address);
            void read_u16(crd514_kd::slaves::t slave, uint16_t first_address, uint16_t* data, unsigned int len);
            uint32_t read_u32(crd514_kd::slaves::t slave, uint16_t address);
    };
}
