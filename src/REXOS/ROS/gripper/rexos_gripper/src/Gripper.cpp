/**
 * @file Gripper.cpp
 * @brief Gripper watchdog
 * @date Created: 2012-10-17
 *
 * @author Koen Braham
 * @author Peter Markotic
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

#include <rexos_gripper/Gripper.h>

#include <iostream>
#include "ros/ros.h"

#include <rexos_utilities/Utilities.h>
#include <rexos_io/TcpModbusInputOutputController.h>
#include <rexos_io/SimulatedInputOutputController.h>

namespace rexos_gripper {
		
		Gripper::Gripper(std::string equipletName, rexos_datatypes::ModuleIdentifier moduleIdentifier, bool isSimulated, Json::Value node) : 
				watchdogRunning(false), isActivated(false), wasActivated(false), warned(false), overheated(false) {
			readJSONNode(node);
			
			if(isSimulated) {
				ioController = new rexos_io::SimulatedInputOutputController(equipletName, moduleIdentifier);
				valveOutputDevice = new OutputDevice(node, ioController);
			} else {
				ioController = new rexos_io::TcpModbusInputOutputController(node);
				valveOutputDevice = new OutputDevice(node, ioController);
			}
		}

		void Gripper::readJSONNode(const Json::Value node) {
			gripperEnabledMax = node["gripperEnabledMaxSeconds"].asInt() * 1000;
			REXOS_INFO_STREAM("found gripperEnabledMaxSeconds " << gripperEnabledMax);
			gripperEnabledWarning = node["gripperEnabledWarningSeconds"].asInt() * 1000;
			REXOS_INFO_STREAM("found gripperEnabledWarningSeconds " << gripperEnabledWarning);
			gripperEnabledCooldown = node["gripperEnabledCooldownSeconds"].asInt() * 1000;
			REXOS_INFO_STREAM("found gripperEnabledCooldownSeconds " << gripperEnabledCooldown);
			watchdogInterval = node["watchdogInterval"].asInt();
			REXOS_INFO_STREAM("found watchdogInterval " << watchdogInterval);
		}
		

		Gripper::~Gripper() {
			REXOS_INFO_STREAM("~Gripper" << std::endl);
			if (watchdogRunning) {
				stopWatchdog();
			}
		}

		/**
		 * Starts the watchdog thread
		 **/
		void Gripper::startWatchdog(){
			//start watchdog thread
			watchdogRunning = true;
			watchdogThread = new boost::thread(watchdogFunction, this);
		}

		/**
		 * Stops the watchdog thread
		 * Blocks untill thread is stopped
		 **/
		void Gripper::stopWatchdog(){
			//stop watchdog thread
			watchdogRunning = false;
			watchdogThread->interrupt();
			watchdogThread->join();
			delete watchdogThread;
			watchdogThread = NULL;
		}
		
		void Gripper::registerObserver(Observer* o){
			observers.push_back(o);
		}
		
		void Gripper::unregisterObserver(Observer* o){
			observers.erase(std::remove(observers.begin(), observers.end(), o), observers.end());
		}
		
		void Gripper::activate() {
			if (overheated == false) {
				valveOutputDevice->enable();
				isActivated = true;
			} else {
				throw std::runtime_error("Gripper activated while it was overheated");
			}
		}
			
		void Gripper::deactivate() {
			valveOutputDevice->disable();
			isActivated = false;
		}
		
		void Gripper::notifyObservers(Notify n){
			for (auto iter = observers.begin(); iter != observers.end(); ++iter){
				switch(n){
					case Warned:
					(*iter)->notifyWarned();
					break;
					
					case CooledDown:
					(*iter)->notifyCooledDown();
					break;
					
					case Overheated:
					(*iter)->notifyOverheated();
					break;
				}
			}
		}
		
		/**
		 * Watchdog function
		 *
		 * @param device The device to be monitored.
		 **/
		void Gripper::watchdogFunction(Gripper* device) {
			try {
				REXOS_INFO_STREAM("[GRIPPER WATCHDOG] Watchdog started");
				while (device->watchdogRunning) {

					// The watchdog in the IO controller disables pins if there is no communication in X amount of time
					// Spam the IO with the current isActivated to keep the pin alive :)
					// TODO: what is X amount of time? Amount of time is probably about 500ms, still the thread can run at 100ms, this will not reduce the preformance
					if (device->isActivated) {
						device->valveOutputDevice->enable();
					} else {
						device->valveOutputDevice->disable();
					}
					
					// Semi correcting the loop time by calculating the run time of the loop.
					long nextRunTime = rexos_utilities::timeNow() + device->watchdogInterval;

					// The device has been turned on
					if (!device->wasActivated && device->isActivated) {
						device->timeEnabled = rexos_utilities::timeNow();
						device->warned = false;

					// If devices stays on
					} else if (device->wasActivated && device->isActivated) {
						long timeEnabled = rexos_utilities::timeNow() - device->timeEnabled;

						// Test for max time, and close valve when reached.
						if (timeEnabled > device->gripperEnabledMax) {
							REXOS_ERROR_STREAM("[GRIPPER WATCHDOG] Valve open time has reached the limit of " << device->gripperEnabledMax << " milliseconds. Gripper will go in cooldown mode now." << std::endl);
							device->overheated = true;
							device->notifyObservers(Overheated);
							device->timeCooldownStarted = rexos_utilities::timeNow();
							device->valveOutputDevice->disable();
							device->wasActivated = device->isActivated = false;

						// Test for warning time. Send warning to the warning handler.
						} else if (!device->warned && timeEnabled > device->gripperEnabledWarning) {
							REXOS_ERROR_STREAM("[GRIPPER WATCHDOG] Valve open time has reached the warning limit of " << device->gripperEnabledWarning << " milliseconds." << std::endl);
							device->warned = true;
							device->notifyObservers(Warned);
						}

					// If device was cooling down, check if the time has been passed.
					} else if (device->overheated && ((rexos_utilities::timeNow() - device->timeCooldownStarted) > device->gripperEnabledCooldown)) {
						REXOS_ERROR_STREAM("[GRIPPER WATCHDOG] Valve cooled down. Returning to normal mode." << std::endl);
						device->overheated = false;
						device->notifyObservers(CooledDown);
					}
					// Save the original isActivated and wait for the next check.
					device->wasActivated = device->isActivated;
					rexos_utilities::sleep(nextRunTime - rexos_utilities::timeNow());
				}
			} catch (boost::thread_interrupted& ignored) {
				// Ignore interrupt and exit thread.
			}
			REXOS_INFO_STREAM("[GRIPPER WATCHDOG] Watchdog stopped" << std::endl);
		}
}
