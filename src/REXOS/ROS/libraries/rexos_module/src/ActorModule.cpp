#include <rexos_module/ActorModule.h>
namespace rexos_module {
	ActorModule::ActorModule(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, bool isShadow) :
		Module(equipletName, identifier, isSimulated, isShadow), 
		setInstructionActionServer(AbstractModule::nodeHandle, 
				equipletName + "/" + identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + identifier.getSerialNumber() + "/set_instruction", 
				boost::bind(&ActorModule::onSetInstruction, this, _1), 
				false)
	{
		setInstructionActionServer.start();
	}
}
