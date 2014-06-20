#include <rexos_knowledge_database/SupportedMutation.h>
#include <iostream>

namespace rexos_knowledge_database {
	SupportedMutation::SupportedMutation(std::string mutation) : 
		mutation(mutation)
	{
		// nothing to do here
	}
	std::string SupportedMutation::getMutation() const {
		return mutation;
	}
	std::ostream& operator<<(std::ostream& os, const SupportedMutation& obj) {
		os << "{mutation: " << obj.getMutation();
		os << "}";
		return os;
	}
}