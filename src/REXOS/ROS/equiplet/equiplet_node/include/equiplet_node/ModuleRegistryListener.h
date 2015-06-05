/*
 * StateMachineListener.h
 *
 *  Created on: 7 jun. 2013
 *      Author: gerben
 */
#pragma once

#include <rexos_module/ModuleProxy.h>
#include <rexos_module/ModuleInterface.h>
#include <rexos_statemachine/State.h>
#include <rexos_statemachine/Mode.h>
#include <rexos_datatypes/RequiredMutation.h>
#include <rexos_datatypes/SupportedMutation.h>

namespace equiplet_node {

class ModuleRegistryListener{
public:
	virtual void onModuleStateChanged	(rexos_module::ModuleProxy* moduleProxy, rexos_statemachine::State newState, rexos_statemachine::State previousState) = 0;
	virtual void onModuleModeChanged	(rexos_module::ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode) = 0;
	virtual void onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, rexos_datatypes::HardwareStep hardwareStep) = 0;
	virtual void onModuleDied(rexos_module::ModuleProxy* moduleProxy) = 0;
	virtual void onModuleTransitionPhaseCompleted(rexos_module::ModuleProxy* moduleProxy, 
			std::vector<rexos_datatypes::SupportedMutation> gainedSupportedMutations, 
			std::vector<rexos_datatypes::RequiredMutation> requiredMutationsRequiredForNextPhase) = 0;
	virtual void spawnNode(rexos_module::ModuleProxy* moduleProxy) = 0;
	virtual void spawnModel(rexos_module::ModuleProxy* moduleProxy) = 0;
	virtual void removeModel(rexos_module::ModuleProxy* moduleProxy) = 0;
};


}
