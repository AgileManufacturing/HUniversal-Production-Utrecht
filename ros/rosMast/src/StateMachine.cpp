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
	ROS_INFO("State changed message received");
	StateType oldState = currentState;
	int equipletID = msg->equipletID;
	int moduleID = msg->moduleID;
	StateType state = StateType(msg->state);
	if( myequipletid == equipletID && mymoduleid == moduleID) {
		//find transition function

		stateFunctionPtr fptr = lookupTransition(currentState, state);
		if(fptr != NULL) {
			if( ( (this->*fptr) () ) == 0 ) {
				setState(state); //set new state and send message
				ROS_INFO("Function pointer executed succesfully");
			} 
			else {
				// Error so I want back to my current state from state
				ROS_INFO("Error in transitioning to new state");
				stateFunctionPtr fptr = lookupTransition(state, oldState);
				if( ( (this->*fptr) () ) == 0 ) {
					ROS_INFO("Transition back to previous state succesfull");
					setState(rosMast::StateType(oldState));
				}
				else {
					ROS_INFO("Error in trying to transition to lower state afer fail transition");
				}
			}
		} else {
			ROS_INFO("Function pointer NULL");
		}
	}	
}

rosMast::StateMachine::StateMachine(int equipletID, int moduleID) {
	myequipletid = equipletID;
	mymoduleid = moduleID;
	currentState = safe;

	//Gives warning
	StateTransition transitionTable[] = { 
		{safe, standby}, 	//0, 3
		{standby, safe}, 	//3, 0
		{standby, normal}, 	//3, 6
		{normal, standby} 	//6, 3
	};	

	//Must be in sync with transitionTable!!!
	transitionMap[transitionTable[0]] = &StateMachine::transitionSetup;
	transitionMap[transitionTable[1]] = &StateMachine::transitionShutdown; 
	transitionMap[transitionTable[2]] = &StateMachine::transitionStart;
	transitionMap[transitionTable[3]] = &StateMachine::transitionStop;

	//Initialize publisher and subcriber
	ros::NodeHandle nh;
	pub = nh.advertise<rosMast::StateChanged>("equiplet_statechanged", 5);
	sub = nh.subscribe("requestStateChange", 1, &StateMachine::changeState, this);
}

rosMast::StateMachine::stateFunctionPtr rosMast::StateMachine::lookupTransition(StateType curState, StateType desiredState) {
	StateTransition st(curState, desiredState);
	return transitionMap[st];
}


void rosMast::StateMachine::setState(StateType newState) {
	std::cout << "setting state to: " << newState << std::endl;
	currentState = newState;
	rosMast::StateChanged msg;
	msg.equipletID = myequipletid;
	msg.moduleID = mymoduleid;
	msg.state = currentState;
	pub.publish(msg);
}

void rosMast::StateMachine::StateEngine() {
	while(ros::ok()) { 
		ros::spinOnce();
	}
}
