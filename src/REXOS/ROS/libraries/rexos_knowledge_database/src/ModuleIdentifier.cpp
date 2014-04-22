#include <rexos_knowledge_database/ModuleIdentifier.h>
#include <string.h>
namespace rexos_knowledge_database{
	ModuleIdentifier::ModuleIdentifier(std::string manufacturer, std::string typeNumber, std::string serialNumber) :
			ModuleTypeIdentifier(manufacturer, typeNumber), 
			manufacturer(manufacturer), typeNumber(typeNumber), serialNumber(serialNumber)
	{
		// nothing to do here
	}
	std::string ModuleIdentifier::getSerialNumber() const {
		return serialNumber;
	}
	std::string ModuleIdentifier::toString() {
		return "{\"manufacturer\":" + getManufacturer() + "\"typeNumber\":" + getTypeNumber() + "\"serialNumber\":" + getSerialNumber() + "}";
	}
	JSONNode ModuleIdentifier::toJSONObject() {
		JSONNode output;
		output.push_back(JSONNode("manufacturer", getManufacturer()));
		output.push_back(JSONNode("typeNumber", getTypeNumber()));
		output.push_back(JSONNode("serialNumber", getSerialNumber()));
		return output;
	}
	bool ModuleIdentifier::operator==(const ModuleIdentifier& rhs) {
		std::cout << this->getManufacturer() << " and " << rhs.getManufacturer() << " ? " << (this->getManufacturer() == rhs.getManufacturer()) << std::endl;
		std::cout << this->getTypeNumber() << " and " << rhs.getTypeNumber() << " ? " << (this->getTypeNumber() == rhs.getTypeNumber()) << std::endl;
		std::cout << this->getSerialNumber() << " and " << rhs.getSerialNumber() << " ? " << (this->getSerialNumber() == rhs.getSerialNumber()) << std::endl;
		if(
				this->getManufacturer() == rhs.getManufacturer() &&
				this->getTypeNumber() == rhs.getTypeNumber() &&
				this->getSerialNumber() == rhs.getSerialNumber()) {
			return true;
		} else {
			return false;
		}
	}
}