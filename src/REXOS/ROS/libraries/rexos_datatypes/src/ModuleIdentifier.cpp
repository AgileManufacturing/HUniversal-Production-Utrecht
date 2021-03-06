#include <rexos_datatypes/ModuleIdentifier.h>

namespace rexos_datatypes{
	ModuleIdentifier::ModuleIdentifier() :
			ModuleTypeIdentifier("", ""), serialNumber("")
	{
		// nothing to do here
	}
	ModuleIdentifier::ModuleIdentifier(std::string manufacturer, std::string typeNumber, std::string serialNumber) :
			ModuleTypeIdentifier(manufacturer, typeNumber), serialNumber(serialNumber)
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
	bool ModuleIdentifier::operator==(const ModuleIdentifier& rhs) const {
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