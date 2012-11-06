/**
 * @file Gripper.cpp
 * @brief Gripper watchdog
 * @date Created: 2012-10-17
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

#include <iostream>
#include <InputOutput/OutputDevices/Gripper.h>
#include <Utilities/Utilities.h>

namespace InputOutput {
	namespace OutputDevices {
		/**
		 * Constructor for Gripper
		 *
		 * @param ioController Output controller for the ouput device (controller contains the registers)
		 * @param warningHandler Handler to warn when the valve is almost opened for too long.
		 */
		Gripper::Gripper(InputOutputController& ioController, void* gripperNodeObject, watchdogWarningHandler warningHandler) :
				OutputDevice(ioController, GRIPPER_MODBUS_ADRESS, GRIPPER_DEVICE_PIN), warningHandler(warningHandler), gripperNode(gripperNodeObject), watchdogRunning(false), state(false), previousState(false), warned(false), overheated(false) {
		}

		/**
		 * Destructor to interrupt the watchdogThread
		 **/
		Gripper::~Gripper( ) {
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

		/**
		 * Watchdog function
		 *
		 * @param device The device to be monitored.
		 **/
		void Gripper::watchdogFunction(Gripper* device) {
			try {
				std::cout << "[GRIPPER WATCHDOG] Watchdog started" << std::endl;
				while (device->watchdogRunning) {

					// The watchdog in the IO controller disables pins if there is no communication in X amount of time
					// Spam the IO with the current state to keep the pin alive :)
					// TODO: what is X amount of time???
					if (device->state) {
						device->enable();
					} else {
						device->disable();
					}

					// Semi correcting the loop time by calculating the run time of the loop.
					long nextRunTime = Utilities::timeNow() + GRIPPER_TIME_WATCHDOG_INTERVAL;

					// The device has been turned on
					if (!device->previousState && device->state) {
						device->timeEnabled = Utilities::timeNow();
						device->warned = false;

					// If devices stays on
					} else if (device->previousState && device->state) {
						long timeEnabled = Utilities::timeNow() - device->timeEnabled;

						// Test for max time, and close valve when reached.
						if (timeEnabled > GRIPPER_TIME_ENABLED_MAX) {
							std::cerr << "[GRIPPER WATCHDOG] Valve open time has reached the limit of " << GRIPPER_TIME_ENABLED_MAX << " milliseconds. Gripper will go in cooldown mode now." << std::endl;
							device->overheated = true;
							device->timeCooldownStarted = Utilities::timeNow();
							device->disable();
							device->previousState = device->state = false;

						// Test for warning time. Send warning to the warning handler.
						} else if (!device->warned && timeEnabled > GRIPPER_TIME_ENABLED_WARNING) {
							std::cerr << "[GRIPPER WATCHDOG] Valve open time has reached the warning limit of " << GRIPPER_TIME_ENABLED_WARNING << " milliseconds." << std::endl;
							device->warningHandler(device->gripperNode);
							device->warned = true;
						}

					// If device was cooling down, check if the time has been passed.
					} else if (device->overheated && ((Utilities::timeNow() - device->timeCooldownStarted) > GRIPPER_TIME_COOLDOWN)) {
						std::cerr << "[GRIPPER WATCHDOG] Valve cooled down. Returning to normal mode." << std::endl;
						device->overheated = false;
					}

					// Save the original state and wait for the next check.
					device->previousState = device->state;
					Utilities::sleep(nextRunTime - Utilities::timeNow());
				}
			} catch (boost::thread_interrupted& ignored) {
				// Ignore interrupt and exit thread.
			}
			std::cout << "[GRIPPER WATCHDOG] Watchdog stopped" << std::endl;
		}

	}
}
