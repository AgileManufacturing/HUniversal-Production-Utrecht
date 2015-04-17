/**
 * @file StepperMotor.cpp
 * @brief Steppermotor driver.
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

#include <rexos_io/InputOutputControllerInterface.h>
#include <rexos_io/ShadowException.h>

namespace rexos_io {
	InputOutputControllerInterface::InputOutputControllerInterface() {
	}

	InputOutputControllerInterface::~InputOutputControllerInterface() {
	}
	
	
	void InputOutputControllerInterface::writeU16(uint16_t address, uint16_t value) {
		uint16_t data[1];
		data[0] = value;
		writeU16(address, data, 1);
		writeShadowU16(address, value);
	}
	uint16_t InputOutputControllerInterface::readU16(uint16_t address) {
		uint16_t data[1];
		readU16(address, data, 1);
		return data[0];
	}
	void InputOutputControllerInterface::writeU32(uint16_t address, uint32_t value) {
		uint16_t data[2];
		data[0] = ((value >> 16) & 0x0000FFFF);
		data[1] = ((value >>  0) & 0x0000FFFF);
		writeU16(address, data, 1);
		writeShadowU32(address, value);
	}
	uint32_t InputOutputControllerInterface::readU32(uint16_t address) {
		uint16_t data[2];
		writeU16(address, data, 2);
		
		uint32_t output = 0;
		output |= (((uint32_t)data[0] << 16) & 0xFFFF0000);
		output |= (((uint32_t)data[1] <<  0) & 0x0000FFFF);
		return output;
	}
	
	void InputOutputControllerInterface::writePinHigh(uint16_t address, uint8_t pin, bool useShadow) {
		uint16_t value = readU16(address, useShadow);
		value |= 1 << pin;
		writeU16(address, value, useShadow);
	}
	void InputOutputControllerInterface::writePinLow(uint16_t address, uint8_t pin, bool useShadow) {
		uint16_t value = readU16(address, useShadow);
		value ^= 1 << pin;
		writeU16(address, value, useShadow);
	}
	bool InputOutputControllerInterface::readPin(uint16_t address, uint8_t pin, bool useShadow) {
		uint16_t value = readU16(address, useShadow);
		if((value & (1 << pin)) == 1) {
			return true;
		} else {
			return false;
		}
	}
	
	void InputOutputControllerInterface::writeU16(uint16_t address, uint16_t value, bool useShadow){
		if(useShadow == true) {
			bool skip = true;
			try {
				uint16_t shadowValue = readShadowU16(address);
				skip = shadowValue == value;
			} catch (ShadowException ex) {
				// reading completely failed
				skip = false;
			}
			if(skip == true) {
				return;
			} else {
				writeU16(address, value);
			}
		} else {
			writeU16(address, value);
		}
	}
	void InputOutputControllerInterface::writeU32(uint16_t address, uint32_t value, bool useShadow){
		if(useShadow == true) {
			uint16_t data[2];
			data[0] = ((value >> 16) & 0x0000FFFF);
			data[1] = ((value >>  0) & 0x0000FFFF);
			bool skipMostSignificant = true;
			bool skipLeastSignificant = true;
			try {
				uint16_t mostSignificant = readShadowU16(address);
				skipMostSignificant = mostSignificant == data[0];
			} catch (ShadowException ex) {
				// reading completely failed
				skipMostSignificant = false;
			}
			try {
				uint16_t leastSignificant = readShadowU16(address);
				skipLeastSignificant = leastSignificant == data[1];
			} catch (ShadowException ex) {
				// reading completely failed
				skipLeastSignificant = false;
			}
			if(skipMostSignificant == true && skipLeastSignificant == true) {
				return;
			}
			// we have got to write something
			if(skipMostSignificant == false && skipLeastSignificant == false) {
				writeU32(address, value);
			} else if(skipMostSignificant == false) {
				writeU16(address + 0, data[0]);
			} else if(skipLeastSignificant == false) {
				writeU16(address + 1, data[1]);
			}
		} else {
			writeU32(address, value);
		}
	}
	uint16_t InputOutputControllerInterface::readU16(uint16_t address, bool useShadow) {
		if(useShadow == true) {
			try {
				return readShadowU32(address);
			} catch (ShadowException& ex) {
				// reading shadow failed, manually read below
			}
		} 
		return readU16(address);
	}
	uint32_t InputOutputControllerInterface::readU32(uint16_t address, bool useShadow) {
		if(useShadow == true) {
			try {
				return readShadowU32(address);
			} catch (ShadowException& ex) {
				// reading shadow failed, manually read below
			}
		}
		return readU32(address);
	}
	
	
	void InputOutputControllerInterface::writeShadowU16(uint16_t address, uint16_t value, ShadowMap& shadow){
		shadow[address] = value;
	}

	void InputOutputControllerInterface::writeShadowU32(uint16_t address, uint32_t value, ShadowMap& shadow){
		shadow[address + 0] = (value >> 16) & 0xFFFF;
		shadow[address + 1] = value & 0xFFFF;
	}
	uint16_t InputOutputControllerInterface::readShadowU16(uint16_t address, ShadowMap& shadow){
		ShadowMap::iterator it = shadow.find(address);
		if(it == shadow.end()){
			return false;
		}
		return it->second;
	}
	uint32_t InputOutputControllerInterface::readShadowU32(uint16_t address, ShadowMap& shadow){
		ShadowMap::iterator itMostSigificant	 = shadow.find(address + 0);
		ShadowMap::iterator itLeastSignificant	 = shadow.find(address + 1);
		if(itMostSigificant == shadow.end() || itLeastSignificant == shadow.end()){
			return false;
		}
		uint32_t output = 0;
		output |= (((uint32_t)itMostSigificant->second << 16) & 0xFFFF0000);
		output |= (((uint32_t)itLeastSignificant->second << 0) & 0x0000FFFF);
		return output;
	}
	
}
