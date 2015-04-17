/**
 * @file MotorInterface.h
 * @brief DeltaRobot motor interface.
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

#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_io/InputOutputControllerInterface.h>

#include <ros/ros.h>

namespace rexos_io{
	/**
	 * Interface for the deltaronot motors.
	 **/
	class SimulatedInputOutputController : public InputOutputControllerInterface {
	public:
		SimulatedInputOutputController(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier);
		virtual ~SimulatedInputOutputController(void);
		
		void writeU16			(uint16_t firstAddress, uint16_t* data, unsigned int length);
		void readU16			(uint16_t firstAddress, uint16_t* data, unsigned int length);
		
		ShadowMap shadowRegistry;
	protected:
		std::string equipletName;
		rexos_datatypes::ModuleIdentifier identifier;
		ros::NodeHandle nodeHandle;
		std::string servicePath;
		
		virtual void writeShadowU16		(uint16_t address, uint16_t value);
		virtual void writeShadowU32		(uint16_t address, uint32_t value);
		virtual uint16_t readShadowU16	(uint16_t address);
		virtual uint32_t readShadowU32	(uint16_t address);
	};
}
