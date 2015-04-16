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

#include <rexos_utilities/Utilities.h>

using namespace std;

extern "C"{
	#include <modbus/modbus.h>
}

namespace rexos_motor{
	MotorManager::MotorManager(std::vector<MotorInterface*> motors) :
			motors(motors), poweredOn(false) {
	}
	void MotorManager::powerOn(void){
		if(!poweredOn){
			for(uint i = 0; i < motors.size(); ++i){
				motors[i]->powerOn();
			}
		}
		poweredOn = true;
	}
	void MotorManager::powerOnSingleMotor(int motorIndex){
		if(!motors[motorIndex]->isPoweredOn()){
			motors[motorIndex]->powerOn();
		}
	}
	void MotorManager::powerOffSingleMotor(int motorIndex){
		if(motors[motorIndex]->isPoweredOn()){
			motors[motorIndex]->powerOff();
		}
	}

	void MotorManager::powerOff(void){
		if(poweredOn){
			for(uint i = 0; i < motors.size(); ++i){
				motors[i]->powerOff();
			}
		}
		poweredOn = false;
	}
}
