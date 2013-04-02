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

#pragma once
#include <rexos_modbus/ModbusController.h>
#include <cmath>

namespace rexos_motor{
	namespace CRD514KD{
		/**
		 * @var double MOTOR_STEP_ANGLE
		 * The angle of a single motor microstep in radians
		 **/
		const double MOTOR_STEP_ANGLE = (0.072 / 180.) * M_PI;

		/**
		 * @var double MOTOR_FULL_STEP_IN_DEGREES
		 * The angle of a single motor full step in degrees
		 **/
		const double MOTOR_FULL_STEP_IN_DEGREES = 1.8;

		/**
		 * @var double MOTOR_MIN_ACCELERATION
		 * The minimum acceleration in radians per second per second. This same value counts for the minimum deceleration.
		 **/
		const double MOTOR_MIN_ACCELERATION = MOTOR_STEP_ANGLE * 1000;

		/**
		 * @var double MOTOR_MAX_ACCELERATION
		 * The maximum acceleration in radians per second per second. This same value counts for the maximum deceleration.
		 **/
		const double MOTOR_MAX_ACCELERATION = MOTOR_STEP_ANGLE * 1000000;

		/**
		 * @var double MOTOR_MIN_SPEED
		 * The minimum speed in radians per second that the motor can travel at, based on the minimum value in the CRD514KD speed register.
		 **/		
		const double MOTOR_MIN_SPEED = MOTOR_STEP_ANGLE * 1;

		/**
		 * @var double MOTOR_MAX_SPEED
		 * The maximum speed in radians per second that the motor can travel at, based on the maximum value in the CRD514KD speed register.
		 **/
		const double MOTOR_MAX_SPEED = MOTOR_STEP_ANGLE * 500000;

		/**
		 * @var int MOTION_SLOTS_USED
		 * The amount of motion slots being used. This value can be anywhere from 1 to 63. However, there is currently no use for it being any more than 2. 
		 **/
		const int MOTION_SLOTS_USED = 2;

		namespace Slaves{
			/**
			 * CRD514KD slave addresses.
			 **/
			typedef enum _t{
				BROADCAST = 0,
				MOTOR_0 = 1,
				MOTOR_1 = 2,
				MOTOR_2 = 3
			} t;
		}

		namespace Registers{
			/**
			 * CRD514KD registers.
			 **/
			enum _registers{
				// 32-bit.
				OP_POS					= 0x402,
				OP_SPEED				= 0x502,

				// 16-bit.
				OP_POSMODE				= 0x601,
				OP_OPMODE				= 0x701,
				OP_SEQ_MODE				= 0x801,

				// 32-bit.
				OP_ACC					= 0x902,
				OP_DEC					= 0xA02,

				// 16-bit.
				OP_DWELL				= 0xC01,

				// 32-bit.
				CFG_POSLIMIT_POSITIVE	= 0x254,
				CFG_POSLIMIT_NEGATIVE	= 0x256,

				// 16-bit.
				CFG_STOP_ACTION			= 0x202,

				// 32-bit.
				CFG_START_SPEED			= 0x228,

				// 16-bit.
				CLEAR_COUNTER			= 0x04b,
				RESET_ALARM				= 0x040,

				CMD_1					= 0x01E,
				STATUS_1				= 0x020,

				// 16-bit current alarm code.
				PRESENT_ALARM			= 0x100,

				// 32-bit Preset position value argument.
				CFG_PRESET_POSITION		= 0x214,

				// 16-bit Set the preset position to the preset position value argument.
				OP_PRESET_POSITION		= 0x048,

				// 16-bit Sets the software motor limitation (in the motor controller).
				OP_SOFTWARE_OVERTRAVEL	= 0x252
			};
		}

		namespace CMD1Bits{
			/**
			 * Bits of value at address CMD_1.
			 **/
			enum _cmd1_bits{
				START			= (1 << 8),
				STOP			= (1 << 11),
				EXCITEMENT_ON	= (1 << 13)
			};
		}

		namespace Status1Bits{
			/**
			 * Bits of value at address STATUS_1.
			 **/
			enum _status1_bits{
				WARNING	= (1 << 6),
				ALARM	= (1 << 7),
				MOVE	= (1 << 10),
				READY	= (1 << 13)
			};
		}

		namespace RtuConfig{
			/**
			 * @var char DEVICE[]
			 * A char array (c string) holding the address of the modbus rtu.
			 **/
			const char DEVICE[] = "/dev/ttyS0";

			/**
			 * Constants used when constructing a modbus rtu (using modbus_new_rtu).
			 **/
			enum _rtu_config{
				BAUDRATE	= 115200,
                PARITY   = 'E',
				DATA_BITS	= 8,
				STOP_BITS	= 1
			};
		}
	}
}
