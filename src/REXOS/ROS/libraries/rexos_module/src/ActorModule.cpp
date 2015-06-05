#include <rexos_module/ActorModule.h>
namespace rexos_module {
	ActorModule::ActorModule(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, bool isShadow) :
		Module(equipletName, identifier, isSimulated, isShadow), 
		executeHardwareStepServer(AbstractModule::nodeHandle, 
				equipletName + "/" + identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + identifier.getSerialNumber() + "/executeHardwareStep", 
				boost::bind(&ActorModule::onExecuteHardwareStep, this, _1), 
				false)
	{
		executeHardwareStepServer.start();
	}
}
