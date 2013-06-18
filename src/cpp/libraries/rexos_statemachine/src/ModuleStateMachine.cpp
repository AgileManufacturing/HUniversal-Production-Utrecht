#include <rexos_statemachine/ModuleStateMachine.h>

#include <equiplet_node/StateUpdate.h>
#include <equiplet_node/ModeUpdate.h>
#include <equiplet_node/RegisterModule.h>

using namespace rexos_statemachine;

ModuleStateMachine::ModuleStateMachine(std::string moduleName, int equipletId, int moduleId)
:StateMachine(moduleName + "_" + std::to_string(equipletId) + "_" + std::to_string(moduleId))
,moduleName(moduleName)
,moduleId(moduleId)
,equipletId(equipletId)
{
	std::string moduleNamespaceName = moduleName + "_" + std::to_string(equipletId) + "_" + std::to_string(moduleId);
	std::string equipletNamespaceName = "equiplet_" + std::to_string(equipletId);

	//Register module on equiplet
	ros::ServiceClient registerModuleServiceClient = nodeHandle.serviceClient<equiplet_node::RegisterModule>(equipletNamespaceName + "/register_module");
	registerModuleServiceClient.waitForExistence();
	equiplet_node::RegisterModuleRequest req;
	equiplet_node::RegisterModuleResponse res;
	req.name = moduleName;
	req.id = moduleId;
	if(!registerModuleServiceClient.call(req, res)) {
		throw new std::runtime_error("Module registration failed");
	}

	changeStateNotificationClient = nodeHandle.serviceClient<equiplet_node::StateUpdate>(equipletNamespaceName + "/" + moduleNamespaceName + "/state_update");
	changeModeNotificationClient = nodeHandle.serviceClient<equiplet_node::ModeUpdate>(equipletNamespaceName + "/" + moduleNamespaceName + "/mode_update");
	setListener(this);
}

void ModuleStateMachine::onStateChanged() {
	equiplet_node::StateUpdateRequest req;
	equiplet_node::StateUpdateResponse res;
	req.state = getCurrentState();
	changeStateNotificationClient.call(req, res);
}

void ModuleStateMachine::onModeChanged() {
	equiplet_node::ModeUpdateRequest req;
	equiplet_node::ModeUpdateResponse res;
	req.mode = getCurrentMode();
	changeModeNotificationClient.call(req, res);
}
