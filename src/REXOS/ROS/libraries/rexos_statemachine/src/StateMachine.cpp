/**
 * @file StateMachine.h
 * @brief Statemachine Implementation
 * @date Created: 2013-17-04
 *
 * @author Gerben Boot & Joris Vergeer
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

#include "rexos_statemachine/StateMachine.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>

#include <cstdlib>

using namespace rexos_statemachine;

/**
 * Create a stateMachine
 * @param moduleID the unique identifier for the module that implements the statemachine
 **/
StateMachine::StateMachine(std::string nodeName,std::vector<rexos_statemachine::Mode> modes):
		modes(modes),
		currentState(STATE_OFFLINE),
		currentMode(MODE_SERVICE),
		changeStateActionServer(nodeHandle, nodeName + "/change_state", boost::bind(&StateMachine::onChangeStateAction, this, _1), false),
		changeModeActionServer(nodeHandle, nodeName + "/change_mode", boost::bind(&StateMachine::onChangeModeAction, this, _1), false),
		changeStateActionClient(nodeHandle, nodeName + "/change_state"),
		changeModeActionClient(nodeHandle, nodeName + "/change_mode"),
		transitionDeinitializeMethod(boost::bind(&StateMachine::transitionDeinitialize, this)),
		transitionInitializeMethod(boost::bind(&StateMachine::transitionInitialize, this)),
		transitionShutdownMethod(boost::bind(&StateMachine::transitionShutdown, this)),
		transitionSetupMethod(boost::bind(&StateMachine::transitionSetup, this)),
		transitionStopMethod(boost::bind(&StateMachine::transitionStop, this)),
		transitionStartMethod(boost::bind(&StateMachine::transitionStart, this))
{

	StatePair transitionInitializeStatePair(STATE_OFFLINE, STATE_SAFE);
	StatePair transitionSetupStatePair(STATE_SAFE, STATE_STANDBY);
	StatePair transitionStartStatePair(STATE_STANDBY, STATE_NORMAL);
	StatePair transitionStopStatePair(STATE_NORMAL, STATE_STANDBY);
	StatePair transitionShutdownStatePair(STATE_STANDBY, STATE_SAFE);
	StatePair transitionDeinitializeStatePair(STATE_SAFE, STATE_OFFLINE);
	
	Transition* transitionDeinitialize = 	new Transition{transitionDeinitializeMethod, STATE_DEINITIALIZE};
	Transition* transitionInitialize = 		new Transition{transitionInitializeMethod, STATE_INITIALIZE};
	Transition* transitionShutdown = 		new Transition{transitionShutdownMethod, STATE_SHUTDOWN};
	Transition* transitionSetup = 			new Transition{transitionSetupMethod, STATE_SETUP};
	Transition* transitionStop = 			new Transition{transitionStopMethod, STATE_STOP};
	Transition* transitionStart = 			new Transition{transitionStartMethod, STATE_START};

	transitionMap[transitionInitializeStatePair] 		= {transitionInitialize,transitionDeinitialize,transitionInitializeStatePair};
	transitionMap[transitionDeinitializeStatePair] 	= {transitionDeinitialize,NULL,transitionDeinitializeStatePair};
	transitionMap[transitionSetupStatePair] 			= {transitionSetup,transitionShutdown,transitionSetupStatePair};
	transitionMap[transitionShutdownStatePair] 		= {transitionShutdown,NULL,transitionShutdownStatePair};
	transitionMap[transitionStartStatePair] 			= {transitionStart,transitionStop,transitionStartStatePair};
	transitionMap[transitionStopStatePair] 			= {transitionStop,NULL,transitionStopStatePair};

	modePossibleStates[MODE_NORMAL] = 			{					STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 													STATE_SETUP, 	STATE_SHUTDOWN, 	STATE_START, 	STATE_STOP};
	modePossibleStates[MODE_SERVICE] =	 		{STATE_OFFLINE, 	STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 	STATE_INITIALIZE, 	STATE_DEINITIALIZE, 	STATE_SETUP, 	STATE_SHUTDOWN, 	STATE_START, 	STATE_STOP};
	modePossibleStates[MODE_ERROR] = 			{					STATE_SAFE, STATE_STANDBY, 																	STATE_SETUP, 	STATE_SHUTDOWN, 					STATE_STOP};
	modePossibleStates[MODE_CRITICAL_ERROR] = 	{STATE_OFFLINE, 	STATE_SAFE, 															STATE_DEINITIALIZE, 							STATE_SHUTDOWN, 					STATE_STOP};
	modePossibleStates[MODE_E_STOP] = 			{					STATE_SAFE};
	modePossibleStates[MODE_LOCK] = 			{					STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 																										STATE_STOP};
	modePossibleStates[MODE_STEP] = 			{					STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 													STATE_SETUP, 	STATE_SHUTDOWN, 	STATE_START, 	STATE_STOP};

	changeStateActionServer.start();
	changeModeActionServer.start();

}

StateMachine::~StateMachine() {
}

State StateMachine::getCurrentState() {
	return currentState;
}

Mode StateMachine::getCurrentMode() {
	return currentMode;
}

/**
 * method to change te state from a subclass of the statemachine.
 * the method send a _changeState action to itself so the action call hasn't implement in the subclass 
 * @param desiredState is the desiredState
 * @param changeStateActionClient = NULL when not given. By this param it is possilbe to waitForResult and getState when finished(SUCCEEDED/ABORTED)
 **/
void StateMachine::changeState(State desiredState) {
	REXOS_INFO("changeState called with desiredState %s",rexos_statemachine::state_txt[desiredState]);
	ChangeStateGoal goal;
	goal.desiredState = desiredState;
	changeStateActionClient.sendGoal(goal);
}

/**
 * method to change te mode from a subclass of the statemachine.
 * the method send a _changeMode action to itself so the action call hasn't implement in the subclass 
 * @param desiredMode is the desiredMode
 * @param changeModeActionClient = NULL when not given. By this param it is possilbe to waitForResult and getState when finished(SUCCEEDED/ABORTED)
 **/
void StateMachine::changeMode(Mode desiredMode) {
	REXOS_INFO("_changeMode called with desiredMode %s",rexos_statemachine::state_txt[desiredMode]);
	ChangeModeGoal goal;
	goal.desiredMode = desiredMode;
	changeModeActionClient.sendGoal(goal);
}

void StateMachine::onChangeStateAction(const ChangeStateGoalConstPtr& goal){
	if(transitionToState((rexos_statemachine::State) goal->desiredState)) changeStateActionServer.setSucceeded();
	else changeStateActionServer.setAborted();
}

void StateMachine::onChangeModeAction(const ChangeModeGoalConstPtr& goal){
	if(transitionToMode( (rexos_statemachine::Mode) goal->desiredMode)) changeModeActionServer.setSucceeded();
	else changeModeActionServer.setAborted();
}

/**
 * Callback for the requestStateChange topic
 * Will lookup the transition function and execute it
 * @param request Contains the params for the state change
 * @param response Will tell if the state transition was succesfull for the state change
 **/
bool StateMachine::transitionToState(rexos_statemachine::State newState) {
	TransitionMap::iterator it = transitionMap.find(StatePair(currentState, newState));
	if (it == transitionMap.end()) {
		return false;
	}

	ChangeStateEntry changeStateEntry = it->second;

	if (!statePossibleInMode(changeStateEntry.transition->transitionState, currentMode) )
		return false;
	
	// set the currentState to the transitionState
	_setState(changeStateEntry.transition->transitionState);
	
	bool transitionIsSuccesfull = changeStateEntry.transition->transitionMethod();
	
	
	if (transitionIsSuccesfull == true){
		// transition succeeded
		_setState(changeStateEntry.statePair.second);
	} else {
		// transition failed, attempt to revert by calling the abort transition
		REXOS_WARN_STREAM("Transition from " << changeStateEntry.statePair.first << " to " << changeStateEntry.statePair.second << " failed");
		if(changeStateEntry.abortTransition == NULL) {
			throw std::logic_error("Trying to access a null abortTransitions state");
		}
		_setState(changeStateEntry.abortTransition->transitionState);
		bool abortTransitionIsSuccesfull = changeStateEntry.abortTransition->transitionMethod();
		
		if (abortTransitionIsSuccesfull == true){
			// abort transition succeeded, revert to original state
			_setState(changeStateEntry.statePair.first);
		} else {
			// abort transition failed, TODO what to do now?
			REXOS_ERROR_STREAM("Abort transition from " << changeStateEntry.statePair.first << " to " << changeStateEntry.statePair.second << " failed");
		}
	}
	
	_forceToAllowedState();
	
	// if change state was successful, we should be in the second state
	return currentState == it->first.second;
}

bool StateMachine::transitionToMode(Mode newMode) {
	bool succeeded = false;
	for(Mode mode : modes){
		if(mode == newMode)
			succeeded = true;
	}

	if(!succeeded)
		return false;

	_setMode(newMode);
	if(newMode == MODE_E_STOP)
		_setState(STATE_SAFE);
	
	_forceToAllowedState();
	return true;
}

bool StateMachine::statePossibleInMode(State state, Mode mode) {
	std::vector<State> States = modePossibleStates[mode];
	for (uint i = 0; i < States.size(); i++) {
		if (States[i] == state)
			return true;
	}
	return false;
}

void StateMachine::_forceToAllowedState() {
	while (!statePossibleInMode(currentState, currentMode)) {
		switch (currentState) {
		case STATE_NORMAL:
			transitionToState(STATE_STANDBY);
			break;
		case STATE_STANDBY:
			transitionToState(STATE_SAFE);
			break;
		case STATE_SAFE:
			transitionToState(STATE_OFFLINE);
			break;
		default:
			break;
		}
	}
}

void StateMachine::_setState(rexos_statemachine::State state) {
	currentState = state;
	onStateChanged(state);
}

void StateMachine::_setMode(rexos_statemachine::Mode mode) {
	currentMode = mode;
	onModeChanged(mode);
}

