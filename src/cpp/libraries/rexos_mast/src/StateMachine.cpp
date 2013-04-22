/**
 * @file StateMachine.h
 * @brief Mast Implementation
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

#include "rexos_mast/StateMachine.h"

/**
 * Create a stateMachine
 * @param equipletID the unique identifier for the equiplet
 * @param moduleID the unique identifier for the module that implements the statemachine
 **/
rexos_mast::StateMachine::StateMachine(int equipletID, int moduleID):
	transitionMap(),
	stateUpdateServer(),
	moduleErrorServer(),
	stateChangeRequestClient(),
	currentState(safe),
	equipletID(equipletID),
	moduleID(moduleID){

	StateTransition transitionTable[] = {StateTransition(safe, standby), StateTransition(standby, safe), StateTransition(standby, normal), StateTransition(normal,standby)};

	// Must be in sync with transitionTable!
	transitionMap[transitionTable[0]] = &StateMachine::transitionSetup;
	transitionMap[transitionTable[1]] = &StateMachine::transitionShutdown; 
	transitionMap[transitionTable[2]] = &StateMachine::transitionStart;
	transitionMap[transitionTable[3]] = &StateMachine::transitionStop;

	// Initialize publisher and subcriber
	ros::NodeHandle nodeHandle;
	std::stringstream stringStream;
	stringStream << equipletID;
	std::string str = stringStream.str();
	stateUpdateServer = nodeHandle.serviceClient<rexos_mast::StateUpdate>("StateUpdate_" + str);
	moduleErrorServer = nodeHandle.serviceClient<rexos_mast::ErrorInModule>("ModuleError_" + str);
	stringStream << "_" << moduleID;
	str = stringStream.str();
	stateChangeRequestClient = nodeHandle.advertiseService("RequestStateChange_" + str, &StateMachine::changeState, this);
}

/**
 * Callback for the requestStateChange topic
 * Will lookup the transition function and execute it
 * @param request Contains the params for the state change
 * @param response Will tell if the state transition was succesfull for the state change
 **/
bool rexos_mast::StateMachine::changeState(rexos_mast::StateChange::Request &request, rexos_mast::StateChange::Response &response){
	// decode msg and read variables
	ROS_INFO("Request Statechange message received");
	StateType desiredState = StateType(request.desiredState);
	
	if(executeTransition(desiredState) == 0) {
		response.executed = true;
	} else {
		response.executed = false;	
	}
	return true;
}

/** 
 * Will lookup the function pointer for a transition and execute the transition function
 * @param desiredState State you want to transition too
 **/
int rexos_mast::StateMachine::executeTransition(rexos_mast::StateType desiredState){
	// save the old state
	StateType oldState = currentState;
	// Lookup transition function ptr
	stateFunctionPtr fptr = lookupTransition(currentState, desiredState);
	if(fptr != NULL) {
		if( ( (this->*fptr) () ) == 0 ) {
			ROS_INFO("Function pointer executed successfully");
			setState(desiredState);
			ROS_INFO("State update successfully");
			return 0;
		} 
		else {
			ROS_INFO("Error in transitioning to new state");
			stateFunctionPtr fptr = lookupTransition(desiredState, oldState);
			if( ( (this->*fptr) () ) == 0 ) {
				ROS_INFO("Transition back to previous state successful");
				setState(oldState);
			}
			else {
				ROS_INFO("Error in transition to old state afer failure in transition");
			}
		}
	} else {
		ROS_INFO("Function pointer NULL, no function found in lookup table");
	}
	return -1;
}


/**
 * Lookup function for the function pointer to the transition function	
 * @param currentState the currentState of the equiplet
 * @param desiredState the desired state
 * @return The pointer to the transition function, will be NULL when there is no function in lookup table
 **/
rexos_mast::StateMachine::stateFunctionPtr rexos_mast::StateMachine::lookupTransition(StateType currentState, StateType desiredState){
	StateTransition st(currentState, desiredState);
	return transitionMap[st];
}

/**
 * Sets the private variable currentState and will send a message over the stateChanged topic
 * @param newState the new state of the machine
 **/
void rexos_mast::StateMachine::setState(StateType newState){
	ROS_INFO("Setting state to: %d", newState);
	currentState = newState;
	rexos_mast::StateUpdate msg;
	msg.request.state.moduleID = this->moduleID;
	msg.request.state.newState = currentState;
	stateUpdateServer.call(msg);
}

/**
 * Send an error message over equiplet_modulerror topic
 * @param errorCode represents an error code that can be looked up in the database
 **/
void rexos_mast::StateMachine::sendErrorMessage(int errorCode){
	rexos_mast::ErrorInModule msg;
	msg.request.moduleError.moduleID = this->moduleID;
	msg.request.moduleError.errorCode = errorCode;
	if(moduleErrorServer.call(msg)) {
		executeTransition(rexos_mast::StateType(msg.response.state.newState));
	}
}