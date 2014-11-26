#include <rexos_module/Module.h>
#include <string>

namespace rexos_module {
	Module::Module(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier) :
		AbstractModule(equipletName, identifier),
		rexos_statemachine::ModuleStateMachine(equipletName, identifier, false),
		transitionActionClient(AbstractModule::nodeHandle, advertisementPath + "transition") 
	{
		init();
	}
	Module::Module(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, ActorModule* actorModule) :
		AbstractModule(equipletName, identifier),
		rexos_statemachine::ModuleStateMachine(equipletName, identifier, true),
		transitionActionClient(AbstractModule::nodeHandle, advertisementPath + "transition") 
	{
		init();
	}
	void Module::init() {
		std::string moduleNamespaceName = identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + identifier.getSerialNumber();
		
		REXOS_INFO_STREAM("binding A on " << (AbstractModule::equipletName + "/bond")<< " id " << moduleNamespaceName);
		bond = new rexos_bond::Bond(AbstractModule::equipletName + "/bond", moduleNamespaceName, this);
		bond->start();
	}
	
	void Module::onBondCallback(rexos_bond::Bond* bond, Event event) {
		if(event == FORMED) {
			REXOS_INFO("Bond has been formed");
		} else {
			REXOS_WARN("Bond has been broken, initiate gracefull shutdown");
			setInError();
			changeState(rexos_statemachine::STATE_OFFLINE);
		}
	}
}