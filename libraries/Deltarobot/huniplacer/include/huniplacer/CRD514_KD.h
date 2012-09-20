//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer
// File:           CRD514_KD.h
// Description:    CRD514 KD constants
// Author:         Lukas Vermond & Kasper van Nieuwland
// Notes:          
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
#include <cmath>

/**
 * @brief holds various CRD514_KD related constants
 **/
namespace crd514_kd
{
    const double MOTOR_STEP_ANGLE = (0.072 / 180.) * M_PI;
    

    /// @brief crd514_kd slave addresses
    namespace slaves
    {
        typedef enum _t
        {
            BROADCAST = 0,
            MOTOR_1 = 1,
            MOTOR_2 = 2,
            MOTOR_3 = 3
        } t;
    }

    /// @brief crd514_kd registers
    namespace registers
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
            STATUS_1                = 0x020  //16-bit
        };
    }

    /// @brief bits of value at address CMD_1
    namespace cmd1_bits
    {
        enum _cmd1_bits
        {
            START         = (1 << 8),
            STOP          = (1 << 11),
            EXCITEMENT_ON = (1 << 13)
        };
    }

    /// @brief bits of value at address STATUS_1
    namespace status1_bits
    {
        enum _status1_bits
        {
            WARNING = (1 << 6),
            ALARM   = (1 << 7),
            MOVE    = (1 << 10),
            READY   = (1 << 13)
        };
    }

    /// @brief constants used when constructing a modbus rtu (using modbus_new_rtu)
    namespace rtu_config
    {
    	const char DEVICE[] = "/dev/ttyS0";
    	enum _rtu_config
    	{
    		BAUDRATE = 115200,
    		PARITY	 = 'N',
    		DATA_BITS = 8,
    		STOP_BITS = 1
    	};
    }
}
