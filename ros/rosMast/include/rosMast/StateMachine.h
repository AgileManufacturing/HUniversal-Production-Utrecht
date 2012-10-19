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
#include "rosMast/StateChanged.h"
#include "rosMast/States.h"
#include <boost/unordered_map.hpp>

#define TRANSITIONMAP_SIZE 4

namespace rosMast {
	struct StateTransition 
	{
		StateTransition() { }
		StateTransition(StateType src, StateType des) {
			SourceState = src;
			DestinationState = des;
		}
		StateType SourceState;
		StateType DestinationState;

		friend bool operator < (const StateTransition& id1, const StateTransition &other) 
		{
			if(id1.SourceState == other.SourceState) {
				return id1.DestinationState < other.DestinationState;
			}				
			return id1.SourceState < other.SourceState;
		}
	};
	
	class StateMachine {
		//Function pointer to transitionFunction or StateFunction
		typedef int (StateMachine::*stateFunctionPtr)();
		
		public:
			StateMachine(int equipletID, int moduleID);
			//Key type = transition state like Setup, t
			std::map<StateTransition, stateFunctionPtr> transitionMap;
		
			virtual int transitionSetup() = 0;
			virtual int transitionShutdown() = 0;
			virtual int transitionStart() = 0;
			virtual int transitionStop() = 0;

			void lock() 	{ locked = true; }
			void unlock() 	{ locked = false;}

			int getState() { return currentState; }

			void StateEngine();
			void changeState(const rosMast::StateChangedPtr &msg);
			void setState(StateType newState);
			stateFunctionPtr lookupTransition(StateType currentState, StateType desiredState);
			StateType lookupState(int state);
		
		protected:
			bool locked;
			StateType currentState;

			ros::Publisher pub;
			ros::Subscriber sub;

		private:		
			int myequipletid;
			int mymoduleid;

		
	};

}

#endif 