#ifndef MASTNODE_H
#define MASTNODE_H

#include "ros/ros.h"
#include <Mast/StateMachine.h>

class MastNode : StateMachine {
	public:
		MastNode() : StateMachine() { StateEngine(); }

	private:
		//State functions
		int stateSafe();
		int stateStandby();
		int stateNormal();

		int transitionSetup();
		int transitionStart();
		int transitionStop();
		int transitionShutdown();
};

#endif