/**
 * @file States.h
 * @brief States for  in module
 * @date Created: 2013-21-03
 * StateBlackboard
 * @author Tommas Bakker
 *
 * @section LICENSE
 * License: newBSD
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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

#include <rexos_knowledge_database/ModuleIdentifier.h>
#include <rexos_knowledge_database/RequiredMutation.h>
#include <rexos_knowledge_database/SupportedMutation.h>

namespace rexos_knowledge_database{
	class TransitionPhase {
	private:
		rexos_knowledge_database::ModuleTypeIdentifier moduleTypeIdentifier;
		int phase;
		std::vector<rexos_knowledge_database::RequiredMutation> requiredMutations;
		std::vector<rexos_knowledge_database::SupportedMutation> supportedMutations;
	public:
		TransitionPhase(rexos_knowledge_database::ModuleTypeIdentifier moduleTypeIdentifier, int phase, 
				std::vector<rexos_knowledge_database::RequiredMutation> requiredMutations, 
				std::vector<rexos_knowledge_database::SupportedMutation> supportedMutations);
		
		rexos_knowledge_database::ModuleTypeIdentifier getModuleTypeIdentifier() const;
		int getPhase() const;
		std::vector<rexos_knowledge_database::RequiredMutation> getRequiredMutations() const;
		std::vector<rexos_knowledge_database::SupportedMutation> getSupportedMutations() const;
	};
	std::ostream& operator<<(std::ostream& os, const TransitionPhase& obj);
}