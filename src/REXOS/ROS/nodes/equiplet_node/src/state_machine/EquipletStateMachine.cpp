#include <equiplet_node/state_machine/EquipletStateMachine.h>
#include <equiplet_node/CyclicDependencyException.h>

#include <rexos_knowledge_database/ModuleType.h>

using namespace equiplet_node;

EquipletStateMachine::EquipletStateMachine(std::string equipletName):
	StateMachine(equipletName,
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
	moduleRegistry(equipletName),
	desiredState(rexos_statemachine::STATE_NOSTATE)
{
	moduleRegistry.setModuleRegistryListener(this);
	moduleRegistry.setNewRegistrationsAllowed(true);
}

EquipletStateMachine::~EquipletStateMachine(){
}

void EquipletStateMachine::changeModuleStates(rexos_statemachine::State desiredState){
	ROS_INFO("\n\n\n\n\n\n\n\n\n\n");
		currenntlySupportedMutations = new std::vector<rexos_knowledge_database::SupportedMutation>();
		pendingTransitionPhases = new std::map<ModuleProxy*, std::vector<rexos_knowledge_database::RequiredMutation>>();

	std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	if(desiredState == rexos_statemachine::STATE_STANDBY && 
			this->getCurrentState() == rexos_statemachine::STATE_SETUP) {
		ROS_INFO("A");
		// we are calibrating the modules, use the required and supported calibration mutations to determine the right order.
		ROS_INFO("B");
		try{
			std::vector<std::vector<rexos_knowledge_database::TransitionPhase>> calibrationSteps = calculateOrderOfCalibrationSteps();
			for (int i = 0; i < modules.size(); i++) {
				ROS_INFO("C");
				modules[i]->changeState(desiredState);
			}
			ROS_INFO("D");
		} catch(CyclicDependencyException ex) {
			ROS_WARN("Cyclic dependency detected");
		}
		return;
	}
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(desiredState);
	}
	ROS_INFO("E");
}

void EquipletStateMachine::onStateChanged(rexos_statemachine::State state){
	ROS_INFO("State Changed to %s",rexos_statemachine::state_txt[getCurrentState()]);
}

void EquipletStateMachine::onModeChanged(rexos_statemachine::Mode mode){
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
			moduleProxy->getModuleIdentifier().toString().c_str(),
			rexos_statemachine::state_txt[previousState],
			rexos_statemachine::state_txt[newState]);
	if(rexos_statemachine::is_transition_state[getCurrentState()] && allModulesInDesiredState(desiredState) ) {
		condit.notify_one();
		delete pendingTransitionPhases;
		delete currenntlySupportedMutations;
	}
}

void EquipletStateMachine::onModuleModeChanged(ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode){
	ROS_INFO("Module Mode Changed received from %s a mode change from %s to %s",
			moduleProxy->getModuleIdentifier().toString().c_str(),
			rexos_statemachine::mode_txt[previousMode],
			rexos_statemachine::mode_txt[newMode]);
	if(newMode > getCurrentMode()){
		changeMode(newMode);
	}
}

void EquipletStateMachine::onModuleDied(ModuleProxy* moduleProxy){
	ROS_ERROR("module has died!");
}

void EquipletStateMachine::onModuleTransitionPhaseCompleted(ModuleProxy* moduleProxy, 
		std::vector<rexos_knowledge_database::SupportedMutation> gainedSupportedMutations, 
		std::vector<rexos_knowledge_database::RequiredMutation> requiredMutationsRequiredForNextPhase) {
	ROS_INFO_STREAM("Module " << moduleProxy->getModuleIdentifier() << " done with transition");
	for(int i = 0; i < gainedSupportedMutations.size(); i++) {
		currenntlySupportedMutations->push_back(gainedSupportedMutations.at(i));
	}
	// insertion will be automatically performed when key does not exists, see http://en.cppreference.com/w/cpp/container/map/operator_at
	(*pendingTransitionPhases)[moduleProxy].clear();
	for(int i = 0; i < requiredMutationsRequiredForNextPhase.size(); i++) {
		(*pendingTransitionPhases)[moduleProxy].push_back(requiredMutationsRequiredForNextPhase.at(i));
	}
	
	
	for(std::map<ModuleProxy*, std::vector<rexos_knowledge_database::RequiredMutation>>::iterator it = pendingTransitionPhases->begin(); 
			it != pendingTransitionPhases->end(); it++) {
		bool canContinue = areAllRequiredMutationsAvailiable(it->second, *currenntlySupportedMutations);
		if(canContinue == true) {
			ROS_INFO_STREAM("Module " << it->first->getModuleIdentifier() << " can now continue");
			it->first->goToNextTransitionPhase();
			// TODO is this safe?
			pendingTransitionPhases->erase(it);
		}
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

bool EquipletStateMachine::transitionInitialize(){
	ROS_INFO( "transitionInitialize called");
	changeModuleStates(rexos_statemachine::STATE_SAFE);
	
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_SAFE)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	
	return true;
}
bool EquipletStateMachine::transitionDeinitialize(){
	ROS_INFO( "transitionDeinitialize called");
	changeModuleStates(rexos_statemachine::STATE_OFFLINE);
	
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_OFFLINE)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	
	return true;
}
bool EquipletStateMachine::transitionSetup(){
	ROS_INFO( "transitionSetup called");
	moduleRegistry.setNewRegistrationsAllowed(false);

	changeModuleStates(rexos_statemachine::STATE_STANDBY);
	
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_STANDBY)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	
	return true;
}

bool EquipletStateMachine::transitionShutdown(){
	ROS_INFO( "transitionShutdown called");
	changeModuleStates(rexos_statemachine::STATE_SAFE);

	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_SAFE)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	moduleRegistry.setNewRegistrationsAllowed(true);
	return true;
}

bool EquipletStateMachine::transitionStart(){
	ROS_INFO( "transitionStart called");
	return true;
}

bool EquipletStateMachine::transitionStop(){
	ROS_INFO( "transitionStop called");
	return true;
}

std::vector<std::vector<rexos_knowledge_database::TransitionPhase>> EquipletStateMachine::calculateOrderOfCalibrationSteps() {
	std::vector<ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	std::vector<rexos_knowledge_database::TransitionPhase> pendingTransitionPhases;
	std::vector<rexos_knowledge_database::SupportedMutation> availiableSupportedMutations;
	for(int i = 0; i < modules.size(); i++) {
		rexos_knowledge_database::ModuleType moduleType = rexos_knowledge_database::ModuleType(modules.at(i)->getModuleIdentifier());
		std::vector<rexos_knowledge_database::TransitionPhase> transitionPhases =  moduleType.getTransitionPhases();
		
		// add all to the pendingTransitionPhases
		for(int j = 0; j < transitionPhases.size(); j++) {
			pendingTransitionPhases.push_back(transitionPhases.at(j));
			ROS_INFO_STREAM(transitionPhases.at(j));
		}
	}
	
	std::vector<std::vector<rexos_knowledge_database::TransitionPhase>> output;
	// actually resolve the graph
	while(pendingTransitionPhases.size() != 0) {
		ROS_INFO("New round");
		std::vector<rexos_knowledge_database::TransitionPhase> currentRound;
		
		std::vector<rexos_knowledge_database::TransitionPhase>::iterator it = pendingTransitionPhases.begin();
		while(it != pendingTransitionPhases.end()) {
			ROS_INFO_STREAM("-" << *it);
			bool areAllRequiredMutationsAreAvailiable = areAllRequiredMutationsAvailiable(
					it->getRequiredMutations(), availiableSupportedMutations);
			ROS_INFO_STREAM("-reqava " << areAllRequiredMutationsAreAvailiable);
			if(areAllRequiredMutationsAreAvailiable == true) {
				ROS_INFO("a");
				// we can perform this calibrationStep
				currentRound.push_back(*it);
				ROS_INFO("a");
				// we just removed an item from the list, but the erase gives an iterator to the next item
				it = pendingTransitionPhases.erase(it);
				ROS_INFO("a");
			} else {
				ROS_INFO("b");
				// just try the next one
				it++;
			}
		}
		ROS_INFO("c");
		if(currentRound.size() == 0) {
			// there was a cyclic dependency (or a depencency is just not supported) as no new transition phase could be performed
			// add the remaining pendingTransitionPhases to the output in sequentional order
			for(int i = 0; i < pendingTransitionPhases.size(); i++) {
				std::vector<rexos_knowledge_database::TransitionPhase> transitionPhasesToAdd;
				transitionPhasesToAdd.push_back(pendingTransitionPhases.at(i));
				output.push_back(transitionPhasesToAdd);
			}
			throw CyclicDependencyException(output);
		} else {
			// we succesfully completed a round, add the gained transition phases to the availiable list
			for(int i = 0; i < currentRound.size(); i++) {
				std::vector<rexos_knowledge_database::SupportedMutation> supportedMutations = currentRound.at(i).getSupportedMutations();
				for(int j = 0; j < supportedMutations.size(); j++) {
					availiableSupportedMutations.push_back(supportedMutations.at(j));
				}
			}
		}
	}
	ROS_INFO("done");
	return output;
}
bool EquipletStateMachine::areAllRequiredMutationsAvailiable(std::vector<rexos_knowledge_database::RequiredMutation> requiredMutations, 
		std::vector<rexos_knowledge_database::SupportedMutation> supportedMutations) {
	bool areAllRequiredMutationsAreAvailiable = true;
	for(int j = 0; j < requiredMutations.size(); j++) {
		rexos_knowledge_database::RequiredMutation requiredMutation = requiredMutations.at(j);
		
		bool isCurrentRequiredMutationsAreAvailiable = false;
		for(int k = 0; k < supportedMutations.size(); k++) {
			if(requiredMutation == supportedMutations.at(k)) isCurrentRequiredMutationsAreAvailiable = true;
		}
		ROS_INFO_STREAM("-reqcurrava " << isCurrentRequiredMutationsAreAvailiable);
		if(isCurrentRequiredMutationsAreAvailiable == false) {
			areAllRequiredMutationsAreAvailiable = false;
		}
	}
	return areAllRequiredMutationsAreAvailiable;
}