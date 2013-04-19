/**
 * @file MotorManager.cpp
 * @brief Motor management for concurrent movement.
 * @date Created: 2012-10-02
 *
 * @author Koen Braham
 * @author Dennis Koole
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

#include <rexos_motor/MotorManager.h>
#include <rexos_motor/CRD514KD.h>
#include <rexos_motor/MotorException.h>

extern "C"{
	#include <modbus/modbus.h>
}

namespace rexos_motor{
	/**
	 * Powers on all motors by doing a broadcast to turn on all excitement for the motors.
	 **/
	void MotorManager::powerOn(void){
		if(!poweredOn){
			for(int i = 0; i < numberOfMotors; ++i){
				motors[i]->powerOn();
			}
		}
		poweredOn = true;
	}

	/**
	 * Powers off all motors by broadcasting a clear for the excitement.
	 **/
	void MotorManager::powerOff(void){
		if(poweredOn){
			for(int i = 0; i < numberOfMotors; ++i){
				motors[i]->powerOff();
			}
		}
		poweredOn = false;
	}

	/**
	 * Start simultaneously movement of all motors
	 **/
	void MotorManager::startMovement(int motionSlot){
		if(!poweredOn){
			throw MotorException("motor manager is not powered on");
		}

		// Execute motion.
		motors[0]->waitTillReady();
		motors[1]->waitTillReady();
		motors[2]->waitTillReady();

		modbus->writeU16(CRD514KD::Slaves::BROADCAST, CRD514KD::Registers::CMD_1, motionSlot | CRD514KD::CMD1Bits::EXCITEMENT_ON | CRD514KD::CMD1Bits::START);
		modbus->writeU16(CRD514KD::Slaves::BROADCAST, CRD514KD::Registers::CMD_1, CRD514KD::CMD1Bits::EXCITEMENT_ON);

		motors[0]->updateAngle();
		motors[1]->updateAngle();
		motors[2]->updateAngle();
	}
}
