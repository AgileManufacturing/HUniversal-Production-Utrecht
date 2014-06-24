/**
 * @file EquipletStateMachine.h
 * @brief 
 * @date Created: 2013-06-25
 *
 * @author Gerben Boot & Joris Vergeer
 *
 * @section LICENSE
 * License: newBSD
 *
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

#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

#include <rexos_statemachine/StateMachine.h>
#include <rexos_statemachine/Listener.h>

#include <rexos_blackboard_cpp_client/BlackboardCppClient.h>
#include <rexos_blackboard_cpp_client/BlackboardSubscriber.h>

#include <equiplet_node/ModuleRegistry.h>
#include <equiplet_node/ModuleProxy.h>

#include <equiplet_node/scada/EquipletScada.h>

#include <rexos_knowledge_database/RequiredMutation.h>
#include <rexos_knowledge_database/SupportedMutation.h>
#include <rexos_knowledge_database/TransitionPhase.h>

namespace equiplet_node{

class EquipletStateMachine : 
	public rexos_statemachine::StateMachine, 
	ModuleRegistryListener {

public:
	EquipletStateMachine(std::string name);

	virtual ~EquipletStateMachine();

protected:
	virtual void onStateChanged(rexos_statemachine::State state);
	virtual void onModeChanged(rexos_statemachine::Mode mode);

	void onModuleStateChanged(ModuleProxy* moduleProxy,rexos_statemachine::State newState, rexos_statemachine::State previousState);

	void onModuleModeChanged(ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode);

	void onModuleDied(ModuleProxy* moduleProxy);
	
	void onModuleTransitionPhaseCompleted(ModuleProxy* moduleProxy, 
			std::vector<rexos_knowledge_database::SupportedMutation> gainedSupportedMutations, 
			std::vector<rexos_knowledge_database::RequiredMutation> requiredMutationsRequiredForNextPhase);
private:
	bool allModulesInDesiredState(rexos_statemachine::State desiredState);

	void changeModuleStates(rexos_statemachine::State desiredState);

	virtual bool transitionInitialize();
	virtual bool transitionDeinitialize();
	virtual bool transitionSetup();
	virtual bool transitionShutdown();
	virtual bool transitionStart();
	virtual bool transitionStop();

protected:
	equiplet_node::ModuleRegistry moduleRegistry; 

private:
	rexos_statemachine::State desiredState;

	boost::condition_variable condit;
	boost::mutex mutexit;
private:
	std::vector<std::vector<rexos_knowledge_database::TransitionPhase>> calculateOrderOfCalibrationSteps();
	bool areAllRequiredMutationsAvailiable(std::vector<rexos_knowledge_database::RequiredMutation> requiredMutations, 
			std::vector<rexos_knowledge_database::SupportedMutation> supportedMutations);
	std::vector<rexos_knowledge_database::SupportedMutation>* currenntlySupportedMutations;
	std::map<ModuleProxy*, std::vector<rexos_knowledge_database::RequiredMutation>>* pendingTransitionPhases;
};

}
