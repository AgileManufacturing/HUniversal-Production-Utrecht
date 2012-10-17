/**
* @file StateMachine.h
* @brief Mast Implementation
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

#include "rosMast/StateMachine.h"

void rosMast::StateMachine::changeState(const rosMast::StateChangedPtr &msg) {
	//decode msg
	int equipletID = msg->equipletID;
	int moduleID = msg->moduleID;
	StateType state = lookupState(msg->state);
	//StateType test = state;
	if( myequipletid == equipletID && mymoduleid == moduleID) {
		//find transition function
		stateFunctionPtr fptr = lookupTransition(currentState, state);
		if(fptr != NULL)
			(this->*fptr)();
	}
	
}

rosMast::StateMachine::StateMachine(int ival) {
	currentState = safe;
	interval = ival;

	// Initialize the state map array
	stateMap[0] = &StateMachine::stateSafe;
	stateMap[1] = &StateMachine::transitionSetup;
	stateMap[2] = &StateMachine::transitionShutdown; 
	stateMap[3] = &StateMachine::stateStandby;
	stateMap[4] = &StateMachine::transitionStart;
	stateMap[5] = &StateMachine::transitionStop;
	stateMap[6] = &StateMachine::stateNormal; 

	//Initialize publisher and subcriber
	ros::NodeHandle nh;
	pub = nh.advertise<rosMast::StateChanged>("equiplet_statechanged", 5);
	sub = nh.subscribe("requestStateChange", 1, &StateMachine::changeState, this);
}

rosMast::StateMachine::stateFunctionPtr rosMast::StateMachine::lookupTransition(StateType currentState, StateType desiredState) {
	return NULL;
}


void rosMast::StateMachine::setState(StateType newState) {
	currentState = newState;
	//send update over publisher
}

rosMast::StateType rosMast::StateMachine::lookupState(int state) {
	switch(state) {
		case safe:
			return safe;
		case standby:
			return standby;
		case normal:
			return normal;
		default:
			return nostate;
	}
}

void rosMast::StateMachine::StateEngine() {
	while(true) { 
		//execute state function
		(this->*stateMap[currentState])();
	}
}

int main() {
	return 0;
}
