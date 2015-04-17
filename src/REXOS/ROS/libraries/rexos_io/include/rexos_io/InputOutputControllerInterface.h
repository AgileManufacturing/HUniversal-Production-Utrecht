#pragma once

#include <stdint.h>
#include <map>

namespace rexos_io{
	/**
	 * Typedef for a shadowMap registers spread over multiple slaves.
	 * Key is slave address. 64bit for multiple slaves.
	 **/
	typedef std::map<uint32_t, uint16_t> ShadowMap;
	
	class InputOutputControllerInterface{
	protected:
		InputOutputControllerInterface();
		virtual ~InputOutputControllerInterface(void);
		
	protected:
		virtual void readU16		(uint16_t firstAddress, uint16_t* data, unsigned int length) = 0;
		virtual void writeU16		(uint16_t firstAddress, uint16_t* data, unsigned int length) = 0;
		void writeU16				(uint16_t address, uint16_t value);
		uint16_t readU16			(uint16_t address);
		void writeU32				(uint16_t address, uint32_t value);
		uint32_t readU32			(uint16_t address);
	public:
		void writePinHigh			(uint16_t address, uint8_t pin, bool useShadow);
		void writePinLow			(uint16_t address, uint8_t pin, bool useShadow);
		bool readPin				(uint16_t address, uint8_t pin, bool useShadow);
		
		void writeU16				(uint16_t address, uint16_t value, bool useShadow);
		void writeU32				(uint16_t address, uint32_t value, bool useShadow);
		uint16_t readU16			(uint16_t address, bool useShadow);
		uint32_t readU32			(uint16_t address, bool useShadow);
	protected:
		/**
		 * Writes a 16-bit value to a shadow register.
		 **/
		void writeShadowU16		(uint16_t address, uint16_t value, ShadowMap& shadow);
		/**
		 * Writes a 32-bit value to a shadow register.
		 **/
		void writeShadowU32		(uint16_t address, uint32_t value, ShadowMap& shadow);
		/**
		 * Reads a 16-bit shadow register.
		 **/
		uint16_t readShadowU16	(uint16_t address, ShadowMap& shadow);
		/**
		 * Reads a 32-bit shadow register.
		 **/
		uint32_t readShadowU32	(uint16_t address, ShadowMap& shadow);
		
		/**
		 * Writes a 16-bit value to a shadow register.
		 **/
		virtual void writeShadowU16		(uint16_t address, uint16_t value) = 0;
		/**
		 * Writes a 32-bit value to a shadow register.
		 **/
		virtual void writeShadowU32		(uint16_t address, uint32_t value) = 0;
		/**
		 * Reads a 16-bit shadow register.
		 **/
		virtual uint16_t readShadowU16	(uint16_t address) = 0;
		/**
		 * Reads a 32-bit shadow register.
		 **/
		virtual uint32_t readShadowU32	(uint16_t address) = 0;
	};
}
