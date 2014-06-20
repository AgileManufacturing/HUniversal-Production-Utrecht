#include <rexos_knowledge_database/RequiredMutation.h>
#include <iostream>

namespace rexos_knowledge_database {
	RequiredMutation::RequiredMutation(std::string mutation, bool isOptional) : 
		SupportedMutation(mutation), isOptional(isOptional)
	{
		// nothing to do here
	}
	bool RequiredMutation::getIsOptional() const {
		return isOptional;
	}
	bool RequiredMutation::operator==(SupportedMutation& rhs) const {
		if(rhs.getMutation() != this->getMutation()) return false;
		return true;
	}
	std::ostream& operator<<(std::ostream& os, const RequiredMutation& obj) {
		os << "{mutation: " << obj.getMutation();
		os << " isOptional: " << obj.getIsOptional();
		os << "}";
		return os;
	}
}