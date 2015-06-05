#include <rexos_module/Module.h>
#include <string>

namespace rexos_module {
	Module::Module(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, bool isShadow) :
		AbstractModule(equipletName, identifier),
		rexos_statemachine::ModuleStateMachine(equipletName, identifier, false),
		transitionActionClient(AbstractModule::nodeHandle, advertisementPath + "transition"),
		isSimulated(isSimulated),
		isShadow(isShadow)
	{
		init();
	}
	Module::Module(std::string equipletName, rexos_datatypes::ModuleIdentifier identifier, bool isSimulated, bool isShadow, ActorModule* actorModule) :
		AbstractModule(equipletName, identifier),
		rexos_statemachine::ModuleStateMachine(equipletName, identifier, true),
		transitionActionClient(AbstractModule::nodeHandle, advertisementPath + "transition"),
		isSimulated(isSimulated),
		isShadow(isShadow)
	{
		init();
	}
	void Module::init() {
		std::string moduleNamespaceName = identifier.getManufacturer() + "/" + identifier.getTypeNumber() + "/" + identifier.getSerialNumber();
		
		REXOS_INFO_STREAM("binding A on " << (AbstractModule::equipletName + "/bond")<< " id " << moduleNamespaceName);
		bond = new bond::Bond(AbstractModule::equipletName + "/bond", moduleNamespaceName, 
				boost::bind(&Module::onBondBrokenCallback, this), 
				boost::bind(&Module::onBondFormedCallback, this));
		bond->start();
	}
	
	void Module::onBondBrokenCallback() {
		REXOS_WARN("Bond has been broken, initiate gracefull shutdown");
		setInError();
		changeState(rexos_statemachine::STATE_OFFLINE);
	}
	void Module::onBondFormedCallback() {
		REXOS_INFO("Bond has been formed");
	}
}
