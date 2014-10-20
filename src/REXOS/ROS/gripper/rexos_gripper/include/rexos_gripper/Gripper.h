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
#ifndef GRIPPER_H
#define GRIPPER_H

#include <rexos_gripper/OutputDevice.h>
#include "rexos_logger/rexos_logger.h"
//#include "REXOS/nodes/gripper/gripper_node/include/gripper_node/Observer.h"
#include "rexos_gripper/Observer.h"

#include <jsoncpp/json/value.h>
#include <vector>
#include <typeinfo>
#include <string>

namespace rexos_gripper {
		/**
		 * Gripper device
		 * Gripper valve could be overheated. A watchdog is running to check the valve is not opened for too long.
		 **/
		class Gripper: public OutputDevice {

			/**
			 * @typedef void (*watchdogWarningHandler)(void* object);
			 * Notifies the handler when valve open time > GRIPPER_TIME_ENABLED_WARNING
			 **/
			typedef void (*watchdogWarningHandler)(void* object);
			std::vector<Observer* > observers;
			
		public:
			Gripper(Json::Value node, void* GripperNode, watchdogWarningHandler warningHandler);
			
			//InputOutputController* ioController, void* GripperNode, watchdogWarningHandler warningHandler);
			virtual ~Gripper();

			void startWatchdog();
			void stopWatchdog();			
			void registerObserver(Observer* o);			
			void unregisterObserver(Observer* o);
			
		private:
			void readJSONNode(Json::Value node);
			
			int gripperEnabledMax;
			int gripperEnabledWarning;
			int gripperEnabledCooldown;
			int watchdogInterval;
			
			/**
			 * @var boost::thread* watchdogThread
			 * The thread that checks the watchdog
			 **/
			boost::thread* watchdogThread;

			/**
			 * @var watchdogWarningHandler warningHandler
			 * The callback function when the gripper is powered on too long
			 **/
			watchdogWarningHandler warningHandler;

			/**
			 * @var void* gripperNode
			 * The object pointer for the object that contains the member callback function
			 **/
			void* gripperNode;

			/**
			 * @var bool watchdogRunning
			 * bool that says wether the watchdog is running
			 **/
			bool watchdogRunning;

			/**
			 * @var bool isActivated
			 * Determens whether the gripper is activated
			 **/
			bool isActivated;
			
			/**
			 * @var bool wasActivated
			 * bool that says whether the gripper was activated before the current state
			 **/
			bool wasActivated;
			
			/**
			 * @var bool warned
			 * Flag whether the warning function has been called after the last warning timeout
			 **/
			bool warned;

			/**
			 * @var bool overheated
			 * Flag whether the valve of the gripper is overheated. This occurs when the enabled max timeout is reached.
			 **/
			bool overheated;

			/**
			 * @var unsigned long timeEnabled
			 * Timestamp of the moment when the gripper was enabled.
			 **/
			unsigned long timeEnabled;

			/**
			 * @var unsigned long timeCooldownStarted
			 * Timestamp of the moment when cooldown period has started.
			 **/
			unsigned long timeCooldownStarted;

			static void watchdogFunction(Gripper* device);

			void notifyObservers();
		};
	}

#endif
