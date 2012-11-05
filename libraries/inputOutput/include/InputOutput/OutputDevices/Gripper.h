/**
* @file OutputDevice.h
* @brief Output device interface
* @date Created: 2012-10-16
*
* @author Koen Braham
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

#include <InputOutput/OutputDevice.h>
#include <boost/thread.hpp>

namespace InputOutput{
	namespace OutputDevices{

		/**
	 	 * Gripper device
	 	 * Gripper valve could be overheated. A watchdog is running to check the valve is not opened for too long.
	 	 **/
		class Gripper : public OutputDevice {

		/**
		 * @typedef
		 * Notifies the handler when valve open time > GRIPPER_TIME_ENABLED_WARNING
		 **/
    	typedef void (*watchdogWarningHandler)();

		public:
			// Config parameters
			/**
			 * @var static int GRIPPER_MODBUS_ADRESS
			 * Register containing the bit (port) for the gripper device
			 * TODO: Should be moved into a dynamic location? QRCODE / Database?
			 **/
 			const static int GRIPPER_MODBUS_ADRESS = 8001;

 			/**
 			 * @var static int GRIPPER_DEVICE_PIN
 			 * Pin (port / bit) of the gripper device
 			 * TODO: Should be moved into a dynamic location? QRCODE / Database?
 			 **/
			const static int GRIPPER_DEVICE_PIN = 0;

			/**
			 * @section time
			 * Timeouts settings for the watchdog. All times are in milliseconds.
			 **/

			/**
			 * @var static int GRIPPER_TIME_ENABLED_MAX
			 * Maximum time for the gripper valve to be opened
			 **/
			const static int GRIPPER_TIME_ENABLED_MAX = 60 * 1000;

			/**
			 * @var static int GRIPPER_TIME_ENABLED_WARNING
			 * Maximum time before a warning is thrown
			 **/
			const static int GRIPPER_TIME_ENABLED_WARNING = 50 * 1000;

			/**
			 * @var static int GRIPPER_TIME_COOLDOWN
			 * Cooldown time for the gripper valve
			 **/
			const static int GRIPPER_TIME_COOLDOWN = 3 * 60 * 1000;

			/**
			 * @var static int GRIPPER_TIME_WATCHDOG_INTERVAL
			 * Watchdog loop interval.
			 * TODO: tune for optimal performance?
			 **/
			const static int GRIPPER_TIME_WATCHDOG_INTERVAL = 100;

			Gripper(InputOutputController& ioController, watchdogWarningHandler warningHandler);
			virtual ~Gripper();

			void grab(){
				state = true;
			}

			void release(){
				state = false;
			}

		private:
			boost::thread* watchdogThread;
			watchdogWarningHandler warningHandler;
			bool watchdogRunning;

			bool state, previousState, warned, overheated;

			unsigned long timeEnabled;
			unsigned long timeCooldownStarted;

			static void watchdogFunction(Gripper* device);
	 	};
	}
}
