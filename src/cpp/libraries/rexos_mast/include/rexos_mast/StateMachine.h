/**
 * @file StateMachine.h
 * @brief Interface for MAST in module
 * @date Created: 2012-10-12
 *
 * @author Arjan Groenewegen
 *
 * @section LICENSE
 * License: newBSD
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
#include "rexos_mast/State.h"
#include "rexos_mast/ModuleError.h"
#include "rexos_mast/ErrorInModule.h"
#include "rexos_mast/StateChange.h"
#include "rexos_mast/StateUpdate.h"
#include "rexos_mast/ModuleError.h"
#include "rexos_mast/States.h"

/**
 * The size of the Transition table
 **/
#define TRANSITION_TABLE_SIZE 4

namespace rexos_mast{
	/**
	 * Struct for a StateTransition
	 * Used to build a array of transitions so they can be mapped to function pointers
	 **/
	struct StateTransition{

		/**
		 * Constructor for StateTransition
		 **/
		StateTransition(StateType src, StateType des) :
			sourceState(src),
			destinationState(des){}

		/**
		 * @var StateType sourceState
		 * The original state
		 **/
		StateType sourceState;

		/**
		 * @var StateType destinationState
		 * Destination state for transition
		 **/
		StateType destinationState;

		/**
		 * Required for building a map, as thats a sorted data structure
		 **/
		friend bool operator < (const StateTransition& id1, const StateTransition &other){
			if(id1.sourceState == other.sourceState){
				return id1.destinationState < other.destinationState;
			}
			return id1.sourceState < other.sourceState;
		}
	};

	/**
	 * The StateMachine
	 **/
	class StateMachine{
		/**
		 * @var typedef int (StateMachine::*stateFunctionPtr)()
		 * Function pointer definition for a state transition function
		 **/
		typedef int (StateMachine::*stateFunctionPtr)();

	public:
		StateMachine(int equipletID, int moduleID);

			virtual ~StateMachine(){ };

		/**
		 * Transition from Safe to Standby
		 * @return 0 if everything when succesfull
		 **/
		virtual int transitionSetup() = 0;

		/**
		 * Transition from Standby to Ssafe
		 * @return 0 if everything when succesfull
		 **/
		virtual int transitionShutdown() = 0;

		/**
		 * Transition from Standby to Normal
		 * @return 0 if everything when succesfull
		 **/
		virtual int transitionStart() = 0;

		/**
		 * Transition from Normal to Standby
		 * @return 0 if everything when succesfull
		 **/
		virtual int transitionStop() = 0;

		/**
		 * Get the state of the statemachine
		 * @return the currentState of the machine
		 **/
		StateType getState(){ return currentState; }

		void setState(StateType newState);
		bool changeState(rexos_mast::StateChange::Request &request, rexos_mast::StateChange::Response &response);
		int executeTransition(StateType desiredState);
		
		stateFunctionPtr lookupTransition(StateType currentState, StateType desiredState);
		void sendErrorMessage(int errorCode);

	private:
		/**
		 * @var std::map<StateTransition, stateFunctionPtr> transitionMap;
		 * The map that links a StateTransition struct to a function pointer
		 **/
		std::map<StateTransition, stateFunctionPtr> transitionMap;

		/**
		 * @var ros::ServiceClient stateUpdateServer
		 * The publisher for posting updated state messages
		 **/
		ros::ServiceClient stateUpdateServer;

		/**
		 * @var ros::ServiceClient moduleErrorServer
		 * The publisher for posting error messages
		 **/
		ros::ServiceClient moduleErrorServer;

		/**
		 * @var ros::ServiceServer stateChangeRequestClient
		 * The subscriber for request for state change messages
		 **/
		ros::ServiceServer stateChangeRequestClient;

		/**
		 * @var StateType currentState
		 * The current state of the the state machine
		 **/
		StateType currentState;

		/**
		 * @var int equipletID
		 * The identifier for the equiplet the module (and so the statemachine) belongs to
		 **/
		int equipletID;

		/**
		 * @var int moduleID
		 * The identifier for the module the state machine belongs to
		 **/
		int moduleID;
	};
}
#endif
