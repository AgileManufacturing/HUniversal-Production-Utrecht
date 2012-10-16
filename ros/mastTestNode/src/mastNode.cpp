#include "mastNode.h"


int MastNode::stateSafe() {
	std::cout  << "Safe";
	return 0;
}

int MastNode::stateStandby() {
	std::cout  << "Standby";
	return 0;
}

int MastNode::stateNormal() {
	std::cout  << "Normal";
	return 0;
}

int MastNode::transitionSetup() {
	currentState = setup;
	return 0;
}

int MastNode::transitionStart(){
	return 0;
}

int MastNode::transitionStop() {
	return 0;
}

int MastNode::transitionShutdown() {
	return 0;
}

int main(int argc, char **argv) {
	MastNode mn;
	return 0;
}