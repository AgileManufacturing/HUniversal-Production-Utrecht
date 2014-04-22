#include <rexos_knowledge_database/ModuleTypeIdentifier.h>
namespace rexos_knowledge_database{
	ModuleTypeIdentifier::ModuleTypeIdentifier(std::string manufacturer, std::string typeNumber) :
			manufacturer(manufacturer), typeNumber(typeNumber)
	{
		// nothing to do here
	}
	std::string ModuleTypeIdentifier::getManufacturer() const {
		return manufacturer;
	}
	std::string ModuleTypeIdentifier::getTypeNumber() const {
		return typeNumber;
	}
}