#include "Mast/StateMachine.h"

StateMachine::StateMachine() {
	currentState = safe;
	// Initialize the transitionMap array
	
	transitionMap[0] = &StateMachine::transitionSetup;
	transitionMap[1] = &StateMachine::transitionStart;
	transitionMap[2] = &StateMachine::transitionStop;
	transitionMap[3] = &StateMachine::transitionShutdown; 
	
	stateMap[0] = &StateMachine::stateSafe;
	stateMap[1] = &StateMachine::stateStandby;
	stateMap[2] = &StateMachine::stateNormal; 

	const struct StateTransition TransitionTable[] = {
		{safe, standby},
		{standby, normal},
		{standby, safe},
		{normal, standby}
	};
}

void StateMachine::StateEngine() {
	while(true) { //Koen finds this scary
		//execute state function
		(this->*stateMap[currentState])();
	}
}

