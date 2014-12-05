/**
 * @file DatabaseConnection.h
 * @brief Coordinate system for communication between nodes
 * @date Created: 2012-01-??  TODO: Date
 *
 * @author Tommas Bakker
 * #author Lars Veenendaal
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
 *
 **/

#pragma once

#include <actionlib/server/simple_action_server.h>

#include <rexos_module/ModuleInterface.h>
#include <rexos_module/ModuleProxyListener.h>
#include <rexos_module/TransitionAction.h>
#include <rexos_module/CandidateModules.h>
#include <bondcpp/bond.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_statemachine/StateMachineController.h>

namespace rexos_module {
	typedef actionlib::SimpleActionServer<TransitionAction> TransitionActionServer;
	
	class ModuleProxy : public ModuleInterface,
		public rexos_statemachine::StateMachineController {
	public:
		ModuleProxy(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, ModuleProxyListener* moduleProxyListener);
		~ModuleProxy();
		
		void changeState(rexos_statemachine::State state);
		void changeMode(rexos_statemachine::Mode mode);
		void goToNextTransitionPhase(std::vector<rexos_module::CandidateModules> candidateModules);
		void bind();
	protected:
		void onStateChange(rexos_statemachine::State newState, rexos_statemachine::State previousState);
		void onModeChange(rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode);
		
		void onBondBrokenCallback();
		void onBondFormedCallback();
	private:
		void onModuleTransitionGoalCallback(const rexos_module::TransitionGoalConstPtr& goal);
	private:
		TransitionActionServer transitionActionServer;
		ModuleProxyListener* moduleProxyListener;
		/**
		 * The bond to bind the module with the equiplet
		 **/
		bond::Bond* bond;
		
		/**
		 * Indicates if the proxy has connected with the actual module node
		 **/
		bool connectedWithNode;
		
		boost::condition_variable nodeStartupCondition;
		boost::mutex nodeStartupMutex;
		
		boost::condition_variable transitionPhaseCondition;
		boost::mutex transitionPhaseMutex;
		/**
		 * Indicates if the module is allowed to continue the transition because the required 
		 * mutations for the next phase are avaliable
		 **/
		bool allowedToContinue;
	};
}