#include <rexos_module/ActorModule.h>
namespace rexos_module {
	ActorModule::ActorModule(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier) :
		Module(equipletName, identifier), 
		setInstructionActionServer(AbstractModule::nodeHandle, 
				equipletName + "/" + identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + identifier.getSerialNumber() + "/set_instruction", 
				boost::bind(&ActorModule::onSetInstruction, this, _1), 
				true)
	{
	}
}
