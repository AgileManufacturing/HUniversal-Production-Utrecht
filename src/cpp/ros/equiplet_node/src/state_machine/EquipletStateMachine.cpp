#include <equiplet_node/state_machine/EquipletStateMachine.h>

using namespace equiplet_node;

EquipletStateMachine::EquipletStateMachine(std::string name,int id):
	StateMachine(name,
		{
			rexos_statemachine::MODE_NORMAL, 
			rexos_statemachine::MODE_SERVICE, 
			rexos_statemachine::MODE_ERROR,
			rexos_statemachine::MODE_CRITICAL_ERROR, 
			rexos_statemachine::MODE_E_STOP,
			rexos_statemachine::MODE_LOCK,
			rexos_statemachine::MODE_STEP	
		}
	),
	moduleRegistry(name, id),
	desiredState(rexos_statemachine::STATE_NOSTATE)
{
	rexos_statemachine::StateMachine::setListener(this);

	moduleRegistry.setModuleRegistryListener(this);

	moduleRegistry.setNewRegistrationsAllowed(true);
}

EquipletStateMachine::~EquipletStateMachine(){
}

void EquipletStateMachine::changeModuleStates(rexos_statemachine::State desiredState){
	std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(desiredState);
	}
}

void EquipletStateMachine::onStateChanged(){
	ROS_INFO("State Changed to %s",rexos_statemachine::state_txt[getCurrentState()]);
}

void EquipletStateMachine::onModeChanged(){
	ROS_INFO("Mode Changed to %s",rexos_statemachine::mode_txt[getCurrentMode()]);	
	bool changeModuleModes = false;

	rexos_statemachine::Mode currentMode = getCurrentMode();
	switch(currentMode){
		case rexos_statemachine::MODE_NORMAL:	
		case rexos_statemachine::MODE_SERVICE:
		case rexos_statemachine::MODE_E_STOP:	
			changeModuleModes = true; break;
	}

	if(changeModuleModes){
		std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
		for (int i = 0; i < modules.size(); i++) {
			modules[i]->changeMode(currentMode);
		}
	}
}

void EquipletStateMachine::onModuleStateChanged(ModuleProxy* moduleProxy,rexos_statemachine::State newState, rexos_statemachine::State previousState){
	ROS_INFO("Module State Changed received from %s a state change from %s to %s",
		moduleProxy->getModuleNodeName().c_str(),rexos_statemachine::state_txt[previousState],rexos_statemachine::state_txt[newState]);
	if(rexos_statemachine::is_transition_state[getCurrentState()] && allModulesInDesiredState(desiredState) )
		condit.notify_one();
}

void EquipletStateMachine::onModuleModeChanged(ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode){
	ROS_INFO("Module Mode Changed received from %s a mode change from %s to %s",
		moduleProxy->getModuleNodeName().c_str(),rexos_statemachine::mode_txt[previousMode],rexos_statemachine::mode_txt[newMode]);
	if(newMode > getCurrentMode()){
		changeMode(newMode);
	}
}

bool EquipletStateMachine::allModulesInDesiredState(rexos_statemachine::State desiredState){
	std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	for(int i=0; i < modules.size(); i++){
		if(modules[i]->getCurrentState() != desiredState){
			return false;
		}
	}
	return true;
}

void EquipletStateMachine::transitionSetup(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO( "transitionSetup called");
	moduleRegistry.setNewRegistrationsAllowed(false);

	changeModuleStates(rexos_statemachine::STATE_STANDBY);
	
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_STANDBY)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	
	as->setSucceeded();
}

void EquipletStateMachine::transitionShutdown(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO( "transitionShutdown called");
	changeModuleStates(rexos_statemachine::STATE_SAFE);

	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_SAFE)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	moduleRegistry.setNewRegistrationsAllowed(true);
	as->setSucceeded();
}

void EquipletStateMachine::transitionStart(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO( "transitionStart called");
	as->setSucceeded();
}

void EquipletStateMachine::transitionStop(rexos_statemachine::TransitionActionServer* as){
	ROS_INFO( "transitionStop called");
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_STANDBY)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	as->setSucceeded();
}