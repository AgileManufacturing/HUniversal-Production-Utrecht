#include <rexos_statemachine/ModuleStateMachine.h>

#include <rexos_statemachine_srvs/StateUpdate.h>
#include <rexos_statemachine_srvs/ModeUpdate.h>
#include <rexos_statemachine_srvs/RegisterModule.h>

using namespace rexos_statemachine;

ModuleStateMachine::ModuleStateMachine(std::string moduleName, int equipletId, int moduleId,bool actorModule)
:StateMachine(moduleName + "_" + std::to_string(equipletId) + "_" + std::to_string(moduleId),
		{rexos_statemachine::MODE_NORMAL, 
		rexos_statemachine::MODE_SERVICE, 
		actorModule ? rexos_statemachine::MODE_CRITICAL_ERROR : rexos_statemachine::MODE_ERROR,
		rexos_statemachine::MODE_E_STOP}
	)
,moduleName(moduleName)
,moduleId(moduleId)
,equipletId(equipletId)
,actorModule(actorModule)
,bond(NULL)
{
	std::string moduleNamespaceName = moduleName + "_" + std::to_string(equipletId) + "_" + std::to_string(moduleId);
	std::string equipletNamespaceName = "equiplet_" + std::to_string(equipletId);

	//Register module on equiplet
	ros::ServiceClient registerModuleServiceClient = nodeHandle.serviceClient<rexos_statemachine_srvs::RegisterModule>(equipletNamespaceName + "/register_module");
	registerModuleServiceClient.waitForExistence();
	rexos_statemachine_srvs::RegisterModuleRequest req;
	rexos_statemachine_srvs::RegisterModuleResponse res;
	req.name = moduleName;
	req.id = moduleId;
	if(!registerModuleServiceClient.call(req, res)) {
		throw new std::runtime_error("Module registration failed");
	}

	changeStateNotificationClient = nodeHandle.serviceClient<rexos_statemachine_srvs::StateUpdate>(equipletNamespaceName + "/" + moduleNamespaceName + "/state_update");
	changeModeNotificationClient = nodeHandle.serviceClient<rexos_statemachine_srvs::ModeUpdate>(equipletNamespaceName + "/" + moduleNamespaceName + "/mode_update");
	setListener(this);
	
	ROS_INFO_STREAM("binding A on " << (moduleName + "/bond")<< " id " << std::to_string(moduleId));
	bond = new rexos_bond::Bond(moduleName + "/bond", std::to_string(moduleId), this);
	bond->start();
}

ModuleStateMachine::~ModuleStateMachine(){
	delete bond;
}

void ModuleStateMachine::onStateChanged() {
	rexos_statemachine_srvs::StateUpdateRequest req;
	rexos_statemachine_srvs::StateUpdateResponse res;
	req.state = getCurrentState();
	changeStateNotificationClient.call(req, res);
}

void ModuleStateMachine::onModeChanged() {
	rexos_statemachine_srvs::ModeUpdateRequest req;
	rexos_statemachine_srvs::ModeUpdateResponse res;
	req.mode = getCurrentMode();
	changeModeNotificationClient.call(req, res);
}

void ModuleStateMachine::setInError(){
	if(actorModule)
		changeMode(rexos_statemachine::MODE_CRITICAL_ERROR);
	else
		changeMode(rexos_statemachine::MODE_ERROR);
}
void ModuleStateMachine::onBondCallback(rexos_bond::Bond* bond, Event event){
	if(event == FORMED) {
		ROS_INFO("Bond has been formed");
	} else {
		ROS_WARN("Bond has been broken, initiate gracefull shutdown");
		// TODO: implement gracefull shutdown
	}
}
