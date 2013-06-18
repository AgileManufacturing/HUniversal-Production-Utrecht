/**
 * @file MOSTStateMachine.h
 * @brief Statemachine for MOST in module
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

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <map>
#include <vector>

#include "rexos_statemachine/Transitions.h"
#include "rexos_statemachine/State.h"
#include "rexos_statemachine/Mode.h"
#include "rexos_statemachine/Listener.h"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <rexos_statemachine/ChangeStateAction.h>
#include <rexos_statemachine/ChangeModeAction.h>
#include <rexos_statemachine/TransitionAction.h>

namespace rexos_statemachine {

typedef actionlib::SimpleActionServer<ChangeStateAction> ChangeStateActionServer;
typedef actionlib::SimpleActionServer<ChangeModeAction> ChangeModeActionServer;
typedef actionlib::SimpleActionServer<TransitionAction> TransitionActionServer;
typedef actionlib::SimpleActionClient<TransitionAction> TransitionActionClient;

class StateMachine: public rexos_statemachine::Transitions {
public:

	StateMachine(std::string nodeName);

	virtual ~StateMachine();

	/**
	 * Get the state of the statemachine
	 * @return the currentState of the machine
	 **/
	rexos_statemachine::State getCurrentState();

	rexos_statemachine::Mode getCurrentMode();

	void setListener(Listener* listener);

	void onChangeStateAction(const ChangeStateGoalConstPtr& goal);

	void onChangeModeAction(const ChangeModeGoalConstPtr& goal);

private:
	void onTransitionSetupAction(TransitionActionServer* as);
	void onTransitionShutdownAction(TransitionActionServer* as);
	void onTransitionStartAction(TransitionActionServer* as);
	void onTransitionStopAction(TransitionActionServer* as);

protected:
	ros::NodeHandle nodeHandle;

private:
	bool changeState(rexos_statemachine::State newState);

	bool changeMode(rexos_statemachine::Mode newMode);

	bool statePossibleInMode(rexos_statemachine::State state, rexos_statemachine::Mode modi);

	void _setState(rexos_statemachine::State state);

	void _setMode(rexos_statemachine::Mode state);

	void _forceToAllowedState();

	Listener* listener;

	/**
	 * @var MOSTState currentState
	 * The current state of the the state machine
	 **/
	rexos_statemachine::State currentState;

	/**
	 * @var MOSTMode currentMode
	 * The current modi of the the state machine
	 **/
	rexos_statemachine::Mode currentMode;

	/**
	 * @var map<rexos_statemachine::Mode,rexos_statemachine::State[]> ModePossibleStates
	 * Possible states of all modus
	 **/
	std::map<rexos_statemachine::Mode, std::vector<rexos_statemachine::State> > modePossibleStates;

	/**
	 * @var typedef int (StateMachine::*stateFunctionPtr)()
	 * Function pointer definition for a state transition function
	 **/
	typedef void (StateMachine::*stateFunctionPtr)();

	/**
	 * @var std::map<std::pair<rexos_statemachine::State,MOSTState>, std::pair<stateFunctionPtr,stateFunctionPtr>> transitionMap;
	 * key is a pair from src to destination
	 * value is a pair with:
	 * the key: functionpointer of the transition
	 * the value: functionpointer of the transition while abort
	 **/

	typedef std::pair<rexos_statemachine::State, rexos_statemachine::State> StatePair;

	struct Transition {
		TransitionActionClient *transitionActionClient;
		//stateFunctionPtr transitionFunctionPointer;
		rexos_statemachine::State transitionState;
		//stateFunctionPtr abortTransitionFunctionPointer;
		//rexos_statemachine::State abortTransitionState;
	};

	struct ChangeStateEntry{
		Transition *transition,*abortTransition;
		StatePair statePair;
	};

//	struct transitionMapEntryValue {
//		stateFunctionPtr transitionFunctionPointer;
//		rexos_statemachine::State transitionState;
//		statePair previousNextState;
//		transitionMapEntryValue abortTransition;
//		//stateFunctionPtr abortTransitionFunctionPointer;
//		//rexos_statemachine::State abortTransitionState;
//	};

	typedef std::pair<StatePair, ChangeStateEntry> transitionMapEntry;
	typedef std::map<StatePair, ChangeStateEntry> transitionMapType;
	transitionMapType transitionMap;

	ChangeStateActionServer changeStateActionServer;
	ChangeModeActionServer changeModeActionServer;

	TransitionActionServer transitionSetupServer;
	TransitionActionServer transitionShutdownServer;
	TransitionActionServer transitionStartServer;
	TransitionActionServer transitionStopServer;

	ros::ServiceClient moduleUpdateServiceClient;

	ChangeStateResult changeStateResult;
};

}
#endif
