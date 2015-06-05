#include <rexos_module/AbstractModule.h>

namespace rexos_module {
	AbstractModule::AbstractModule(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier) : 
			rexos_knowledge_database::Module(identifier), 
			rexos_coordinates::Module(this), 
			equipletName(equipletName), 
			identifier(identifier)
	{
		advertisementPath = equipletName + "/" + identifier.getManufacturer() + "/" + 
				identifier.getTypeNumber() + "/" + identifier.getSerialNumber() + "/";
	}
	std::string AbstractModule::getEquipletName() {
		return equipletName;
	}
	rexos_datatypes::ModuleIdentifier AbstractModule::getModuleIdentifier() {
		return identifier;
	}

}
