/**
* @file CRD514KD.h
* @brief CRD514-KD constants.
* @date Created: 2012-10-01
*
* @author 1.0 Lukas Vermond
* @author 1.0 Kasper van Nieuwland
* @author 1.1 Koen Braham
* @author 1.1 Dennis Koole
*
* @section LICENSE
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


#pragma once
#include <ModbusController/ModbusController.h>
#include <cmath>

/**
 * CRD514KDMotorController.h -> holds various CRD514_KD related constants
 **/
namespace Motor
{
    namespace CRD514KD
    {
        const double MOTOR_STEP_ANGLE = (0.072 / 180.) * M_PI;
        const double MOTOR_STEP_IN_DEGREES = 1.8;

        /**
         * CRD514KD slave addresses
         **/
        namespace Slaves
        {
            typedef enum _t
            {
                BROADCAST = 0,
                MOTOR_0 = 1,
                MOTOR_1 = 2,
                MOTOR_2 = 3
            } t;
        }

        /**
         * CRD514KD registers
         **/
        namespace Registers
        {
            enum _registers
            {
                OP_POS                  = 0x402, //32-bit
                OP_SPEED                = 0x502, //32-bit
                OP_POSMODE              = 0x601, //16-bit
                OP_OPMODE               = 0x701, //16-bit
                OP_SEQ_MODE             = 0x801, //16-bit
                OP_ACC                  = 0x902, //32-bit
                OP_DEC                  = 0xA02, //32-bit
                OP_DWELL                = 0xC01, //16-bit
                
                CFG_POSLIMIT_POSITIVE   = 0x254, //32-bit
                CFG_POSLIMIT_NEGATIVE   = 0x256, //32-bit
                CFG_STOP_ACTION         = 0x202, //16-bit
                CFG_START_SPEED         = 0x228, //32-bit
                
                CLEAR_COUNTER           = 0x04b, //16-bit
                RESET_ALARM             = 0x040, //16-bit
                
                CMD_1                   = 0x01E, //16-bit
                STATUS_1                = 0x020, //16-bit

                PRESENT_ALARM           = 0x100, // 16-bit current alarm code.

                //32-bit Preset position value argument
                CFG_PRESET_POSITION = 0x214,

                //16-bit Set the preset position to the preset position value argument
                OP_PRESET_POSITION = 0x048, 

                //16-bit Sets the software motor limitation (in the motor controller)
                OP_SOFTWARE_OVERTRAVEL = 0x252 
            };
        }

        /**
         * Bits of value at address CMD_1
         **/
        namespace CMD1Bits
        {
            enum _cmd1_bits
            {
                START         = (1 << 8),
                STOP          = (1 << 11),
                EXCITEMENT_ON = (1 << 13)
            };
        }

        /**
         * Bits of value at address STATUS_1
         **/
        namespace Status1Bits
        {
            enum _status1_bits
            {
                WARNING = (1 << 6),
                ALARM   = (1 << 7),
                MOVE    = (1 << 10),
                READY   = (1 << 13)
            };
        }

        /** 
         * Constants used when constructing a modbus rtu (using modbus_new_rtu)
         **/
        namespace RtuConfig
        {
            const char DEVICE[] = "/dev/ttyS0";
            enum _rtu_config
            {
                BAUDRATE = 115200,
                PARITY   = 'N',
                DATA_BITS = 8,
                STOP_BITS = 1
            };
        }
    }
}