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
		listener(NULL),
		currentState(STATE_OFFLINE),
		currentMode(MODE_SERVICE),
		modes(modes),
		changeStateActionServer(nodeHandle, nodeName + "/change_state", boost::bind(&StateMachine::onChangeStateAction, this, _1), false),
		changeModeActionServer(nodeHandle, nodeName + "/change_mode", boost::bind(&StateMachine::onChangeModeAction, this, _1), false),
		transitionDeinitializeServer(nodeHandle, nodeName + "/transition_deinitialize", boost::bind(&StateMachine::onTransitionDeinitializeAction, this, &transitionDeinitializeServer), false),
		transitionInitializeServer(nodeHandle, nodeName + "/transition_initialize", boost::bind(&StateMachine::onTransitionInitializeAction, this, &transitionInitializeServer), false),
		transitionSetupServer(nodeHandle, nodeName + "/transition_setup", boost::bind(&StateMachine::onTransitionSetupAction, this, &transitionSetupServer), false),
		transitionShutdownServer(nodeHandle, nodeName + "/transition_shutdown", boost::bind(&StateMachine::onTransitionShutdownAction, this, &transitionShutdownServer), false),
		transitionStartServer(nodeHandle, nodeName + "/transition_start", boost::bind(&StateMachine::onTransitionStartAction, this, &transitionStartServer), false),
		transitionStopServer(nodeHandle, nodeName + "/transition_stop", boost::bind(&StateMachine::onTransitionStopAction, this, &transitionStopServer), false),
		changeStateActionClient(nodeHandle, nodeName + "/change_state"),
		changeModeActionClient(nodeHandle, nodeName + "/change_mode")
{

	StatePair transitionInitializeStatePair(STATE_OFFLINE, STATE_SAFE);
	StatePair transitionSetupStatePair(STATE_SAFE, STATE_STANDBY);
	StatePair transitionStartStatePair(STATE_STANDBY, STATE_NORMAL);
	StatePair transitionStopStatePair(STATE_NORMAL, STATE_STANDBY);
	StatePair transitionShutdownStatePair(STATE_STANDBY, STATE_SAFE);
	StatePair transitionDeinitializeStatePair(STATE_SAFE, STATE_OFFLINE);

	TransitionActionClient* transitionActionClientDeinitialize = new TransitionActionClient(nodeName + "/transition_deinitialize",true);
	TransitionActionClient* transitionActionClientInitialize = new TransitionActionClient(nodeName + "/transition_initialize",true);
	TransitionActionClient* transitionActionClientSetup = new TransitionActionClient(nodeName + "/transition_setup",true);
	TransitionActionClient* transitionActionClientStart = new TransitionActionClient(nodeName + "/transition_start",true);
	TransitionActionClient* transitionActionClientStop = new TransitionActionClient(nodeName + "/transition_stop",true);
	TransitionActionClient* transitionActionClientShutdown = new TransitionActionClient(nodeName + "/transition_shutdown",true);

	Transition* transitionDeinitialize = new Transition{transitionActionClientDeinitialize, STATE_DEINITIALIZE};
	Transition* transitionInitialize = new Transition{transitionActionClientInitialize, STATE_INITIALIZE};
	Transition* transitionShutdown = new Transition{transitionActionClientShutdown, STATE_SHUTDOWN};
	Transition* transitionSetup = new Transition{transitionActionClientSetup, STATE_SETUP};
	Transition* transitionStop = new Transition{transitionActionClientStop, STATE_STOP};
	Transition* transitionStart = new Transition{transitionActionClientStart, STATE_START};

	transitionMap[transitionInitializeStatePair]= {transitionInitialize,transitionDeinitialize,transitionInitializeStatePair};
	transitionMap[transitionDeinitializeStatePair]= {transitionDeinitialize,NULL,transitionDeinitializeStatePair};
	transitionMap[transitionSetupStatePair]= {transitionSetup,transitionShutdown,transitionSetupStatePair};
	transitionMap[transitionShutdownStatePair]= {transitionShutdown,NULL,transitionShutdownStatePair};
	transitionMap[transitionStartStatePair]= {transitionStart,transitionStop,transitionStartStatePair};
	transitionMap[transitionStopStatePair]= {transitionStop,NULL,transitionStopStatePair};

	modePossibleStates[MODE_NORMAL] = 			{					STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 												STATE_SETUP, 	STATE_SHUTDOWN, 	STATE_START, 	STATE_STOP};
	modePossibleStates[MODE_SERVICE] =	 		{STATE_OFFLINE, 	STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 	STATE_INITIALIZE, 	STATE_DEINITIALIZE, 	STATE_SETUP, 	STATE_SHUTDOWN, 	STATE_START, 	STATE_STOP};
	modePossibleStates[MODE_ERROR] = 			{					STATE_SAFE, STATE_STANDBY, 																STATE_SETUP, 	STATE_SHUTDOWN, 					STATE_STOP};
	modePossibleStates[MODE_CRITICAL_ERROR] = 	{STATE_OFFLINE, 	STATE_SAFE, 													STATE_DEINITIALIZE, 					STATE_SHUTDOWN, 					STATE_STOP};
	modePossibleStates[MODE_E_STOP] = 			{					STATE_SAFE};
	modePossibleStates[MODE_LOCK] = 			{					STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 																									STATE_STOP};
	modePossibleStates[MODE_STEP] = 			{					STATE_SAFE, STATE_STANDBY, STATE_NORMAL, 												STATE_SETUP, 	STATE_SHUTDOWN, 	STATE_START, 	STATE_STOP};

	changeStateActionServer.start();
	changeModeActionServer.start();

	transitionInitializeServer.start();
	transitionDeinitializeServer.start();
	transitionSetupServer.start();
	transitionShutdownServer.start();
	transitionStartServer.start();
	transitionStopServer.start();
}

StateMachine::~StateMachine() {
	/*delete transitionActionClientStart;
	delete transitionActionClientStop;
	delete transitionActionClientSetup;
	delete transitionActionClientShutdown;

	delete transitionStart;
	delete transitionStop ;
	delete transitionSetup;
	delete transitionShutdown;*/
}

void StateMachine::onTransitionInitializeAction(TransitionActionServer* as){
	transitionInitialize(as);
}

void StateMachine::onTransitionDeinitializeAction(TransitionActionServer* as){
	transitionDeinitialize(as);
}

void StateMachine::onTransitionSetupAction(TransitionActionServer* as){
	transitionSetup(as);
}

void StateMachine::onTransitionShutdownAction(TransitionActionServer* as){
	transitionShutdown(as);
}

void StateMachine::onTransitionStartAction(TransitionActionServer* as){
	transitionStart(as);
}

void StateMachine::onTransitionStopAction(TransitionActionServer* as){
	transitionStop(as);
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
	ROS_INFO("changeState called with desiredState %s",rexos_statemachine::state_txt[desiredState]);
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
	ROS_INFO("_changeMode called with desiredMode %s",rexos_statemachine::state_txt[desiredMode]);
	ChangeModeGoal goal;
	goal.desiredMode = desiredMode;
	changeModeActionClient.sendGoal(goal);
}

void StateMachine::onChangeStateAction(const ChangeStateGoalConstPtr& goal){
	if(_changeState((rexos_statemachine::State) goal->desiredState))
		changeStateActionServer.setSucceeded();
	else
		changeStateActionServer.setAborted();
}

void StateMachine::onChangeModeAction(const ChangeModeGoalConstPtr& goal){
	if(_changeMode( (rexos_statemachine::Mode) goal->desiredMode))
		changeModeActionServer.setSucceeded();
	else
		changeModeActionServer.setAborted();
}

/**
 * Callback for the requestStateChange topic
 * Will lookup the transition function and execute it
 * @param request Contains the params for the state change
 * @param response Will tell if the state transition was succesfull for the state change
 **/
bool StateMachine::_changeState(rexos_statemachine::State newState) {
	TransitionMap::iterator it = transitionMap.find(StatePair(currentState, newState));
	if (it == transitionMap.end()) {
		return false;
	}

	ChangeStateEntry changeStateEntry = it->second;

	if (!statePossibleInMode(changeStateEntry.transition->transitionState, currentMode) )
		return false;

	
	// set the currentState to the transitionState
	_setState(changeStateEntry.transition->transitionState);
	
	// wait until the transition has completed
	
	
	
	
	
	
	
	
	
	TransitionGoal goal;
	TransitionActionClient* transitionActionClient = changeStateEntry.transition->transitionActionClient;
	transitionActionClient->waitForServer();
	transitionActionClient->sendGoal(goal, 
			actionlib::SimpleActionClient<rexos_statemachine::TransitionAction>::SimpleDoneCallback(), 
			actionlib::SimpleActionClient<rexos_statemachine::TransitionAction>::SimpleActiveCallback(), 
			boost::bind(&StateMachine::onTransitionFeedbackCallback, this, _1));
	transitionActionClient->waitForResult();
	
	if (transitionActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		// transition succeeded
		_setState(changeStateEntry.statePair.second);
	} else if(transitionActionClient->getState() == actionlib::SimpleClientGoalState::ABORTED) {
		// transition failed, attempt to revert by calling the abort transition
		ROS_WARN_STREAM("Transition from " << changeStateEntry.statePair.first << " to " << changeStateEntry.statePair.second << "failed");
		if(changeStateEntry.abortTransition == NULL) {
			throw std::logic_error("Trying to access a null abortTransitions state");
		}
		_setState(changeStateEntry.abortTransition->transitionState);
		TransitionActionClient* abortTransitionActionClient = changeStateEntry.abortTransition->transitionActionClient;
		abortTransitionActionClient->sendGoal(goal);
		abortTransitionActionClient->waitForResult();
		
		if (abortTransitionActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			// abort transition succeeded, revert to original state
			_setState(changeStateEntry.statePair.first);
		} else if(abortTransitionActionClient->getState() == actionlib::SimpleClientGoalState::ABORTED) {
			// abort transition failed, TODO what to do now?
			ROS_ERROR_STREAM("Abort transition from " << changeStateEntry.statePair.first << " to " << changeStateEntry.statePair.second << "failed");
		} else {
			throw std::runtime_error("Recieved a SimpleClientGoalState from abortTransitionActionClient which was not expected");
		}
	} else {
		throw std::runtime_error("Recieved a SimpleClientGoalState from transitionActionClient which was not expected");
	}
	
	_forceToAllowedState();
	
	// if change state was successful, we should be in the second state
	return currentState == it->first.second;
}

bool StateMachine::_changeMode(Mode newMode) {
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
	for (int i = 0; i < States.size(); i++) {
		if (States[i] == state)
			return true;
	}
	return false;
}

void StateMachine::_forceToAllowedState() {
	while (!statePossibleInMode(currentState, currentMode)) {
		switch (currentState) {
		case STATE_NORMAL:
			_changeState(STATE_STANDBY);
			break;
		case STATE_STANDBY:
			_changeState(STATE_SAFE);
			break;
		case STATE_SAFE:
			_changeState(STATE_OFFLINE);
			break;
		default:
			break;
		}
	}
}

void StateMachine::onTransitionFeedbackCallback(const rexos_statemachine::TransitionFeedbackConstPtr& feedback) {
	std::cout << "Processing some feedback" << std::endl;
	rexos_statemachine::ChangeStateFeedback output;
	output.gainedSupportedMutations = feedback->gainedSupportedMutations;
	output.requiredMutationsRequiredForNextPhase = feedback->requiredMutationsRequiredForNextPhase;
	changeStateActionServer.publishFeedback(output);
}

void StateMachine::setListener(Listener* listener) {
	this->listener = listener;
}

void StateMachine::_setState(State state) {
	currentState = state;
	if (listener != NULL) {
		listener->onStateChanged();
	}
}

void StateMachine::_setMode(Mode mode) {
	currentMode = mode;
	if (listener != NULL) {
		listener->onModeChanged();
	}
}
