/**
* @file StateMachine.h
* @brief Interface for MAST in module
* @date Created: 2012-10-12
*
* @author Arjan Groenewegen
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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "ros/ros.h"

#define TRANSITION_MAP_SIZE 4
#define STATE_MAP_SIZE 3

typedef enum {safe = 0, standby = 1, normal = 2, setup = 3, shutdown = 4, start = 5, stop = 6} StateType;

struct StateTransition 
{
	StateTransition() { }
	StateTransition(StateType src, StateType des) {
		SourceState = src;
		DestinationState = des;
	}
	StateType SourceState;
	StateType DestinationState;
};

class StateMachine {
	//Function pointer to transitionFunction or StateFunction
	typedef int (StateMachine::*stateFunctionPtr)();
	
	public:
		StateMachine();

		//Transition functions
		virtual int transitionSetup() = 0;		//Safe to standby transition
		virtual int transitionStart() = 0; 		//Standby to normal transitiion
		virtual int transitionStop() = 0;		//Normal to standby
		virtual int transitionShutdown() = 0;	//standby to safe
		
		//State functions
		virtual int stateSafe() = 0;
		virtual int stateStandby() = 0;
		virtual int stateNormal() = 0;

		void lock() 	{ locked = true; }
		void unlock() 	{ locked = false;}

		int getState() { return currentState; }

		void StateEngine();
		void changeState(StateType requestedState);
		void setState(StateType newState) {
			currentState = newState;
		}
	
	protected:
		bool locked;
		
		stateFunctionPtr transitionMap[TRANSITION_MAP_SIZE];
		stateFunctionPtr stateMap[STATE_MAP_SIZE];
		
		struct StateTransition TransitionTable[TRANSITION_MAP_SIZE];

	private:
		StateType currentState;
};

#endif 