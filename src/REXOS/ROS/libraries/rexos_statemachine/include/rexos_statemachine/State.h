/**
 * @file States.h
 * @brief States for  in module
 * @date Created: 2013-17-03
 *
 * @author Gerben Boot & Joris Vergeer
 *
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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

#ifndef STATES_H
#define STATES_H

#define STATE_COUNT 10

namespace rexos_statemachine{
	/**
	 * @ enum State
	 * The enumeration for the states
	 **/
	typedef enum {
		STATE_OFFLINE = 0,
		STATE_INITIALIZE = 1,
		STATE_DEINITIALIZE = 2,
		STATE_SAFE = 3,
		STATE_SETUP = 4,
		STATE_SHUTDOWN = 5,
		STATE_STANDBY = 6,
		STATE_START = 7,
		STATE_STOP = 8,
		STATE_NORMAL = 9,
		STATE_NOSTATE = 10
	} State;

	/**
	 * @var const char* const state_txt[]
	 * The representation of the states in a char array so its easy to print the enum values instead of the integer
	 **/
	const char* const state_txt[] = { "Offline", "Initialize", "Deinitialize", "Safe", "Setup", "Shutdown", "Standby", "Start", "Stop", "Normal", "no state" };

	const char is_transition_state[] = { 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0 };
}
#endif
