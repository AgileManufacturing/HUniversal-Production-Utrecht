/*
 * StateMachineListener.h
 *
 *  Created on: 7 jun. 2013
 *      Author: gerben
 */
#pragma once

#include "equiplet_node/ModuleProxy.h"
#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>

namespace equiplet_node {

class ModuleRegistryListener{
public:
	virtual void onModuleStateChanged(ModuleProxy* moduleProxy,rexos_statemachine::State newState, rexos_statemachine::State previousState) = 0;
	virtual void onModuleModeChanged(ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode) = 0;
};

}