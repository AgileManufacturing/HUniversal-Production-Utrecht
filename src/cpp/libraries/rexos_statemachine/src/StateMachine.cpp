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
#include <boost/bind.hpp>

#include <cstdlib>

using namespace rexos_statemachine;

/**
 * Create a stateMachine
 * @param moduleID the unique identifier for the module that implements the statemachine
 **/
StateMachine::StateMachine(std::string nodeName) :
		listener(NULL),
		currentState(STATE_SAFE),
		currentMode(MODE_NORMAL),
		changeStateActionServer(nodeHandle, nodeName + "/change_state", boost::bind(&StateMachine::onChangeStateAction, this, _1), false),
		changeModeActionServer(nodeHandle, nodeName + "/change_mode", boost::bind(&StateMachine::onChangeModeAction, this, _1), false),
		transitionSetupServer(nodeHandle, nodeName + "/transition_setup", boost::bind(&StateMachine::onTransitionSetupAction, this, &transitionSetupServer), false),
		transitionShutdownServer(nodeHandle, nodeName + "/transition_shutdown", boost::bind(&StateMachine::onTransitionShutdownAction, this, &transitionShutdownServer), false),
		transitionStartServer(nodeHandle, nodeName + "/transition_start", boost::bind(&StateMachine::onTransitionStartAction, this, &transitionStartServer), false),
		transitionStopServer(nodeHandle, nodeName + "/transition_stop", boost::bind(&StateMachine::onTransitionStopAction, this, &transitionStopServer), false)
{

	StatePair transitionSetupStatePair(STATE_SAFE, STATE_STANDBY);
	StatePair transitionStartStatePair(STATE_STANDBY, STATE_NORMAL);
	StatePair transitionStopStatePair(STATE_NORMAL, STATE_STANDBY);
	StatePair transitionShutdownStatePair(STATE_STANDBY, STATE_SAFE);

	Transition *transitionShutdown = new Transition{new TransitionActionClient(nodeName + "/transition_shutdown",true), STATE_SHUTDOWN};
	Transition *transitionSetup = new Transition{new TransitionActionClient(nodeName + "/transition_setup",true), STATE_SETUP};
	Transition *transitionStop = new Transition{new TransitionActionClient(nodeName + "/transition_stop",true), STATE_STOP};
	Transition *transitionStart = new Transition{new TransitionActionClient(nodeName + "/transition_start",true), STATE_START};

	transitionMap[transitionSetupStatePair]= {transitionSetup,transitionShutdown,transitionSetupStatePair};
	transitionMap[transitionShutdownStatePair]= {transitionShutdown,NULL,transitionShutdownStatePair};
	transitionMap[transitionStartStatePair]= {transitionStart,transitionStop,transitionStartStatePair};
	transitionMap[transitionStopStatePair]= {transitionStop,NULL,transitionStopStatePair};

	modiPossibleStates[MODE_NORMAL] = {STATE_NORMAL,STATE_STANDBY,STATE_SAFE};
	modiPossibleStates[MODE_ERROR] = {STATE_STANDBY,STATE_SAFE};
	modiPossibleStates[MODE_CRITICAL_ERROR] = {STATE_SAFE};
	modiPossibleStates[MODE_E_STOP] = {STATE_SAFE};

	changeStateActionServer.start();
	changeModeActionServer.start();

	transitionSetupServer.start();
	transitionShutdownServer.start();
	transitionStartServer.start();
	transitionStopServer.start();
}

StateMachine::~StateMachine() {
	//TODO pointer values remove
}

void StateMachine::onChangeStateAction(const ChangeStateGoalConstPtr& goal){
	bool b = true;
	switch (goal->desiredState) {
		case rexos_statemachine::STATE_SAFE:
			b = changeState(STATE_SAFE);
			break;
		case rexos_statemachine::STATE_STANDBY:
			b = changeState(STATE_STANDBY);
			break;
		case rexos_statemachine::STATE_NORMAL:
			b = changeState(STATE_NORMAL);
			break;
		default:
			b = false;
		}

	if(b)
		changeStateActionServer.setSucceeded();
	else
		changeStateActionServer.setAborted(changeStateResult);
}
void StateMachine::onChangeModeAction(const ChangeModeGoalConstPtr& goal){
	ChangeModeResult res;
	switch (goal->desiredMode) {
		case rexos_statemachine::MODE_NORMAL:
			res.executed = changeMode(MODE_NORMAL);
			break;
		case rexos_statemachine::MODE_SERVICE:
			res.executed = changeMode(MODE_SERVICE);
			break;
		case rexos_statemachine::MODE_ERROR:
			res.executed = changeMode(MODE_ERROR);
			break;
		case rexos_statemachine::MODE_CRITICAL_ERROR:
			res.executed = changeMode(MODE_CRITICAL_ERROR);
			break;
		case rexos_statemachine::MODE_E_STOP:
			res.executed = changeMode(MODE_E_STOP);
			break;
		default:
			changeModeActionServer.setAborted(res);
		}
	changeModeActionServer.setSucceeded(res);
}

/**
 * Callback for the requestStateChange topic
 * Will lookup the transition function and execute it
 * @param request Contains the params for the state change
 * @param response Will tell if the state transition was succesfull for the state change
 **/
bool StateMachine::changeState(rexos_statemachine::State newState) {
	// decode msg and read variables
	//ROS_INFO("Request Statechange message received");
	if (!statePossibleInMode(newState, currentMode) && newState > currentState )
		return false;

	transitionMapType::iterator it = transitionMap.find(StatePair(currentState, newState));
	if (it == transitionMap.end()) {
		return false;
	}

	ChangeStateEntry changeStateEntry = it->second;
	ROS_INFO("ChangeState from:%d to %d", it->second.statePair.first,it->second.statePair.second);
	_setState(changeStateEntry.transition->transitionState);				//set the currentState on the transitionState
	TransitionActionClient* transitionActionClient = changeStateEntry.transition->transitionActionClient;
	TransitionGoal goal;
	transitionActionClient->sendGoal(goal);
	transitionActionClient->waitForResult();

	if (transitionActionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		//transition succeeded
		_setState(changeStateEntry.statePair.second);
	}else if(currentState != changeStateEntry.abortTransition->transitionState){
		//abort method
		_setState(changeStateEntry.abortTransition->transitionState);
		transitionActionClient = changeStateEntry.abortTransition->transitionActionClient;
		transitionActionClient->sendGoal(goal);
	}else{
		//transition without an abort transition failed such as 'stop/shutdown' transition
	}
	_forceToAllowedState();

	return true;
}

void StateMachine::onTransitionSetupAction(TransitionActionServer* as){
	transitionSetup();
	as->setSucceeded();
}

void StateMachine::onTransitionShutdownAction(TransitionActionServer* as){
	transitionShutdown();
	as->setSucceeded();
}

void StateMachine::onTransitionStartAction(TransitionActionServer* as){
	transitionStart();
	as->setSucceeded();
}

void StateMachine::onTransitionStopAction(TransitionActionServer* as){
	transitionStop();
	as->setSucceeded();
}

State StateMachine::getCurrentState() {
	return currentState;
}

Mode StateMachine::getCurrentMode() {
	return currentMode;
}

bool StateMachine::statePossibleInMode(State state, Mode modi) {
	std::vector<State> States = modiPossibleStates[modi];
	for (int i = 0; i < States.size(); i++) {
		if (States[i] == state)
			return true;
	}
	return false;
}

bool StateMachine::changeMode(Mode newMode) {
	_setMode(newMode);
	_forceToAllowedState();
	return true;
}

void StateMachine::_forceToAllowedState() {
	while (!statePossibleInMode(currentState, currentMode)) {
		switch (currentState) {
		case STATE_NORMAL:
			changeState(STATE_STANDBY);
			break;
		case STATE_STANDBY:
			changeState(STATE_SAFE);
			break;
		}
	}
}

void StateMachine::setListener(Listener* listener) {
	this->listener = listener;
}

void StateMachine::_setState(State state) {
	ROS_INFO("state changed to:%s",state_txt[state]);
	currentState = state;
	if (listener != NULL) {
		listener->onStateChanged();
	}
}

void StateMachine::_setMode(Mode modi) {
	currentMode = modi;
	if (listener != NULL) {
		listener->onModeChanged();
	}
}
