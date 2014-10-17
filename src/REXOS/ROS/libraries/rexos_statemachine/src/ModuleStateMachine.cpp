#include <rexos_statemachine/ModuleStateMachine.h>

#include <rexos_statemachine/StateUpdate.h>
#include <rexos_statemachine/ModeUpdate.h>
#include <rexos_statemachine/RegisterModule.h>

using namespace rexos_statemachine;

ModuleStateMachine::ModuleStateMachine(std::string equipletName, rexos_knowledge_database::ModuleIdentifier moduleIdentifier, bool actorModule) : 
		StateMachine(equipletName + "/" + moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber(),
			{rexos_statemachine::MODE_NORMAL, 
			rexos_statemachine::MODE_SERVICE, 
			actorModule ? rexos_statemachine::MODE_CRITICAL_ERROR : rexos_statemachine::MODE_ERROR,
			rexos_statemachine::MODE_E_STOP}
		), 
		equipletName(equipletName), moduleIdentifier(moduleIdentifier), actorModule(actorModule), bond(NULL),
		transitionActionClient(nodeHandle, equipletName + 
			"/" + moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber() + "/transition") 
		{
	std::string moduleNamespaceName = moduleIdentifier.getManufacturer() + "/" + moduleIdentifier.getTypeNumber() + "/" + moduleIdentifier.getSerialNumber();
	std::string equipletNamespaceName = equipletName;

	//Register module on equiplet
	ros::ServiceClient registerModuleServiceClient = nodeHandle.serviceClient<rexos_statemachine::RegisterModule>(equipletNamespaceName + "/register_module");
	registerModuleServiceClient.waitForExistence();
	rexos_statemachine::RegisterModuleRequest req;
	rexos_statemachine::RegisterModuleResponse res;
	req.manufacturer = moduleIdentifier.getManufacturer();
	req.typeNumber = moduleIdentifier.getTypeNumber();
	req.serialNumber = moduleIdentifier.getSerialNumber();
	if(!registerModuleServiceClient.call(req, res)) {
		throw std::runtime_error("Module registration failed");
	}

	changeStateNotificationClient = nodeHandle.serviceClient<rexos_statemachine::StateUpdate>(equipletNamespaceName + "/" + moduleNamespaceName + "/state_update");
	changeModeNotificationClient = nodeHandle.serviceClient<rexos_statemachine::ModeUpdate>(equipletNamespaceName + "/" + moduleNamespaceName + "/mode_update");
	
	REXOS_INFO_STREAM("binding A on " << (equipletNamespaceName + "/bond")<< " id " << moduleNamespaceName);
	bond = new rexos_bond::Bond(equipletNamespaceName + "/bond", moduleNamespaceName, this);
	bond->start();
}

ModuleStateMachine::~ModuleStateMachine(){
	delete bond;
}

void ModuleStateMachine::onStateChanged(rexos_statemachine::State state) {
	REXOS_WARN("onStateChanged");
	rexos_statemachine::StateUpdateRequest req;
	rexos_statemachine::StateUpdateResponse res;
	req.state = getCurrentState();
	changeStateNotificationClient.call(req, res);
}

void ModuleStateMachine::onModeChanged(rexos_statemachine::Mode mode) {
	REXOS_WARN("onModeChanged");
	rexos_statemachine::ModeUpdateRequest req;
	rexos_statemachine::ModeUpdateResponse res;
	req.mode = getCurrentMode();
	changeModeNotificationClient.call(req, res);
}

void ModuleStateMachine::setInError() {
	if(actorModule)
		changeMode(rexos_statemachine::MODE_CRITICAL_ERROR);
	else
		changeMode(rexos_statemachine::MODE_ERROR);
}
void ModuleStateMachine::onBondCallback(rexos_bond::Bond* bond, Event event){
	if(event == FORMED) {
		REXOS_INFO("Bond has been formed");
	} else {
		REXOS_WARN("Bond has been broken, initiate gracefull shutdown");
		// TODO: implement gracefull shutdown
	}
}
