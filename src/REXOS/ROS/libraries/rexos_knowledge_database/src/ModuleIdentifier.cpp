#include <rexos_knowledge_database/ModuleIdentifier.h>

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
	Json::Value ModuleIdentifier::toJSONObject() {
		Json::Value output;
		output["manufacturer"] = getManufacturer();
		output["typeNumber"] = getTypeNumber();
		output["serialNumber"] = getSerialNumber();
		return output;
	}
	bool ModuleIdentifier::operator==(const ModuleIdentifier& rhs) {
		if(		this->getManufacturer() == rhs.getManufacturer() &&
				this->getTypeNumber() == rhs.getTypeNumber() &&
				this->getSerialNumber() == rhs.getSerialNumber()) {
			return true;
		} else {
			return false;
		}
	}
	std::ostream& operator<<(std::ostream& os, const ModuleIdentifier& obj) {
		os << "manufacturer: " << obj.getManufacturer();
		os << "typeNumber: " << obj.getTypeNumber();
		os << "serialNumber: " << obj.getSerialNumber();
		return os;
	}
}