#include <rexos_datatypes/SupportedMutation.h>
#include <iostream>

namespace rexos_datatypes {
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