#include <rexos_knowledge_database/TransitionPhase.h>

#include <iostream>

namespace rexos_knowledge_database
{
	TransitionPhase::TransitionPhase(rexos_knowledge_database::ModuleTypeIdentifier moduleTypeIdentifier, int phase, 
				std::vector<rexos_knowledge_database::RequiredMutation> requiredMutations, 
				std::vector<rexos_knowledge_database::SupportedMutation> supportedMutations) :
		moduleTypeIdentifier(moduleTypeIdentifier), phase(phase), requiredMutations(requiredMutations), supportedMutations(supportedMutations)
	{
		// nothing to do here
	}
	rexos_knowledge_database::ModuleTypeIdentifier TransitionPhase::getModuleTypeIdentifier() const {
		return moduleTypeIdentifier;
	}
	int TransitionPhase::getPhase() const {
		return phase;
	}
	std::vector<rexos_knowledge_database::RequiredMutation> TransitionPhase::getRequiredMutations() const {
		return requiredMutations;
	}
	std::vector<rexos_knowledge_database::SupportedMutation> TransitionPhase::getSupportedMutations() const {
		return supportedMutations;
	}
	std::ostream& operator<<(std::ostream& os, const TransitionPhase& obj) {
		os << "{moduleTypeIdentifier: " << obj.getModuleTypeIdentifier();
		os << " phase: " << obj.getPhase();
		os << " requiredMutations: [";
		for(int i = 0; i < obj.getRequiredMutations().size(); i++) {
			if(i != 0) os << ", \n";
			os << obj.getRequiredMutations().at(i);
		}
		os << "] supportedMutations: [";
		for(int i = 0; i < obj.getSupportedMutations().size(); i++) {
			if(i != 0) os << ", \n";
			os << obj.getSupportedMutations().at(i);
		}
		os << "]}";
		return os;
	}
}
