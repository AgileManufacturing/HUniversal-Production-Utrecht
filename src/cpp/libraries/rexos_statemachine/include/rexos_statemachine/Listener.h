/*
 * StateMachineListener.h
 *
 *  Created on: 7 jun. 2013
 *      Author: gerben
 */

#ifndef LISTENER_H_
#define LISTENER_H_

namespace rexos_statemachine {

class Listener {
public:
	virtual void onStateChanged() = 0;
	virtual void onModeChanged() = 0;
};

}

#endif /* LISTENER_H_ */
