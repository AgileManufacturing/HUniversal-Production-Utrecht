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

#include "rosMast/StateMachine.h"

/**
 * Create a stateMachine
 * @param the unique identifier for the equiplet
 * @param the unique identifier for the module that implements the statemachine
 **/
rosMast::StateMachine::StateMachine(int equipletID, int moduleID) {
	this->equipletID = equipletID;
	this->moduleID = moduleID;
	currentState = safe;

	// Initialized this way because the other way wont work on older compilers
	StateTransition transitionTable[TRANSITION_TABLE_SIZE];
	transitionTable[0] = StateTransition(safe, standby);	
	transitionTable[1] = StateTransition(standby, safe);	
	transitionTable[2] = StateTransition(standby, normal);		
	transitionTable[3] = StateTransition(normal, standby);		

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
	stateUpdateServer = nodeHandle.serviceClient<rosMast::StateUpdate>("StateUpdate_" + str);
	moduleErrorServer = nodeHandle.serviceClient<rosMast::ErrorInModule>("ModuleError_" + str);
	stateChangeRequestClient = nodeHandle.advertiseService("RequestStateChange_" + str, &StateMachine::changeState, this);
}

/**
 * Callback for the requestStateChange topic
 * Will lookup the transition function and execute it
 * @param request Contains the params for the state change
 * @param response Will tell if the state transition was succesfull for the state change
 **/
bool rosMast::StateMachine::changeState(rosMast::StateChange::Request &request, rosMast::StateChange::Response &response) {
	// save the old state
	StateType oldState = currentState;
	
	// decode msg and read variables
	ROS_INFO("Request Statechange message received");
	int moduleID = request.state.moduleID;
	StateType desiredState = StateType(request.state.newState);
	
	// Check if the message is meant for this StateMachine
	if(this->moduleID == moduleID ) {
		// Lookup transition function ptr
		stateFunctionPtr fptr = lookupTransition(currentState, desiredState);
		if(fptr != NULL) {
			if( ( (this->*fptr) () ) == 0 ) {
				setState(desiredState);
				ROS_INFO("Function pointer executed successfully");
				response.executed = true;
				return true;
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
	} else {
		ROS_INFO("State changerequest not meant for this statemachine");
		ROS_INFO("Statemachine equipletID = %d", this->equipletID);
		ROS_INFO("Statemachine moduleID = %d", this->moduleID);
	}
	response.executed = false;		
	return true;
}


/**
 * Lookup function for the function pointer to the transition function	
 * @param the currentState of the equiplet
 * @param the desired state
 * @return The pointer to the transition function, will be NULL when there is no function in lookup table
 **/
rosMast::StateMachine::stateFunctionPtr rosMast::StateMachine::lookupTransition(StateType curState, StateType desiredState) {
	StateTransition st(curState, desiredState);
	return transitionMap[st];
}

/**
 * Sets the private variable currentState and will send a message over the stateChanged topic
 * @param the new state of the machine
 **/
void rosMast::StateMachine::setState(StateType newState) {
	ROS_INFO("Setting state to: %d", newState);
	currentState = newState;
	rosMast::StateUpdate msg;
	msg.request.state.moduleID = this->moduleID;
	msg.request.state.newState = currentState;
	stateUpdateServer.call(msg);
}

/**
 * Send an error message over equiplet_modulerror topic
 * @param represents an error code that can be looked up in the database
 **/
void rosMast::StateMachine::sendErrorMessage(int errorCode) {
	rosMast::ErrorInModule msg;
	msg.request.moduleError.moduleID = this->moduleID;
	msg.request.moduleError.errorCode = errorCode;
	moduleErrorServer.call(msg);
}

/** 
 * The run part of the state machine
 **/
void rosMast::StateMachine::startStateMachine() {
	while(ros::ok()) { 
		ros::spinOnce();
	}
}
