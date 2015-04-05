#include <equiplet_node/state_machine/EquipletStateMachine.h>

#include <equiplet_node/CyclicDependencyException.h>
#include <node_spawner_node/spawnNode.h>
#include <model_spawner_node/spawnModel.h>
#include <model_spawner_node/removeModel.h>
#include <rexos_knowledge_database/ModuleType.h>

using namespace equiplet_node;

EquipletStateMachine::EquipletStateMachine(std::string equipletName, bool isSimulated):
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
	moduleRegistry(equipletName, this),
	desiredState(rexos_statemachine::STATE_NOSTATE),
	spawnNodeClient(nodeHandle.serviceClient<node_spawner_node::spawnNode>(equipletName + "/spawnNode")),
	spawnModelClient(nodeHandle.serviceClient<model_spawner_node::spawnModel>(equipletName + "/spawnModel")),
	removeModelClient(nodeHandle.serviceClient<model_spawner_node::removeModel>(equipletName + "/removeModel"))
{
	if(isSimulated == true) {
		REXOS_INFO("Spawning models for modules");
		moduleRegistry.spawnModels();
	}
	moduleRegistry.setNewRegistrationsAllowed(true);
}

EquipletStateMachine::~EquipletStateMachine(){
}

void EquipletStateMachine::changeModuleStates(rexos_statemachine::State desiredState) {
	currentlySupportedMutations = new std::map<rexos_datatypes::SupportedMutation, std::vector<rexos_module::ModuleProxy*>>();
	pendingTransitionPhases = new std::map<rexos_module::ModuleProxy*, std::vector<rexos_datatypes::RequiredMutation>>();

	std::vector<rexos_module::ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	if(desiredState == rexos_statemachine::STATE_STANDBY && this->getCurrentState() == rexos_statemachine::STATE_SETUP) {
		// we are calibrating the modules, use the required and supported calibration mutations to determine the right order.
		try{
			std::vector<std::vector<rexos_datatypes::TransitionPhase>> calibrationSteps = calculateOrderOfCalibrationSteps();
			for (int i = 0; i < modules.size(); i++) {
				modules[i]->changeState(desiredState);
			}
		} catch(CyclicDependencyException ex) {
			REXOS_WARN("Cyclic dependency detected");
		}
		return;
	}
	for (int i = 0; i < modules.size(); i++) {
		modules[i]->changeState(desiredState);
	}
}

void EquipletStateMachine::onStateChanged(rexos_statemachine::State state){
	REXOS_INFO("State Changed to %s",rexos_statemachine::state_txt[getCurrentState()]);
}

void EquipletStateMachine::onModeChanged(rexos_statemachine::Mode mode){
	REXOS_INFO("Mode Changed to %s",rexos_statemachine::mode_txt[getCurrentMode()]);
	bool changeModuleModes = false;

	rexos_statemachine::Mode currentMode = getCurrentMode();
	switch(currentMode){
		case rexos_statemachine::MODE_NORMAL:
		case rexos_statemachine::MODE_SERVICE:
		case rexos_statemachine::MODE_E_STOP:
			changeModuleModes = true; break;
	}

	if(changeModuleModes){
		std::vector<rexos_module::ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
		for (int i = 0; i < modules.size(); i++) {
			modules[i]->changeMode(currentMode);
		}
	}
}

void EquipletStateMachine::onModuleStateChanged(rexos_module::ModuleProxy* moduleProxy, 
		rexos_statemachine::State newState, rexos_statemachine::State previousState) {
	if(rexos_statemachine::is_transition_state[getCurrentState()] && allModulesInDesiredState(desiredState) ) {
		condit.notify_one();
		delete pendingTransitionPhases;
		delete currentlySupportedMutations;
	}
}

void EquipletStateMachine::onModuleModeChanged(rexos_module::ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode) {
	if(newMode > getCurrentMode()){
		changeMode(newMode);
	}
}

void EquipletStateMachine::onModuleDied(rexos_module::ModuleProxy* moduleProxy){
	REXOS_ERROR_STREAM("module " << moduleProxy->getModuleIdentifier() << " has died!");
}

void EquipletStateMachine::onModuleTransitionPhaseCompleted(rexos_module::ModuleProxy* moduleProxy, 
		std::vector<rexos_datatypes::SupportedMutation> gainedSupportedMutations, 
		std::vector<rexos_datatypes::RequiredMutation> requiredMutationsRequiredForNextPhase) {
	REXOS_INFO_STREAM("Module " << moduleProxy->getModuleIdentifier() << " is done with transition");
	for(int i = 0; i < gainedSupportedMutations.size(); i++) {
		(*currentlySupportedMutations)[gainedSupportedMutations.at(i)].push_back(moduleProxy);
	}
	// insertion will be automatically performed when key does not exists, see http://en.cppreference.com/w/cpp/container/map/operator_at
	(*pendingTransitionPhases)[moduleProxy].clear();
	for(int i = 0; i < requiredMutationsRequiredForNextPhase.size(); i++) {
		(*pendingTransitionPhases)[moduleProxy].push_back(requiredMutationsRequiredForNextPhase.at(i));
	}
	
	
	for(auto it = pendingTransitionPhases->begin(); it != pendingTransitionPhases->end(); it++) {
		bool canContinue = areAllRequiredMutationsAvailiable(it->second, *currentlySupportedMutations);
		if(canContinue == true) {
			REXOS_INFO_STREAM("Module " << it->first->getModuleIdentifier() << " can now continue");
			
			std::vector<rexos_module::CandidateModules> candidatesForRequiredMutations;
			for(int i = 0; i < it->second.size(); i++) {
				rexos_module::CandidateModules candidatesForRequiredMutation;
				candidatesForRequiredMutation.mutation = it->second.at(i).getMutation();
				for(int j = 0; j < (*currentlySupportedMutations)[it->second.at(i)].size(); j++) {
					rexos_module::ModuleProxy* candidateProxy = (*currentlySupportedMutations)[it->second.at(i)].at(j);
					candidatesForRequiredMutation.manufacturer.push_back(candidateProxy->getModuleIdentifier().getManufacturer());
					candidatesForRequiredMutation.typeNumber.push_back(candidateProxy->getModuleIdentifier().getTypeNumber());
					candidatesForRequiredMutation.serialNumber.push_back(candidateProxy->getModuleIdentifier().getSerialNumber());
				}
				candidatesForRequiredMutations.push_back(candidatesForRequiredMutation);
			}
			it->first->goToNextTransitionPhase(candidatesForRequiredMutations);
			// TODO is this safe?
			pendingTransitionPhases->erase(it);
		}
	}
}
void EquipletStateMachine::spawnNode(rexos_module::ModuleProxy* moduleProxy) {
	ROS_INFO_STREAM("Spawning node for " << moduleProxy->getModuleIdentifier());
	node_spawner_node::spawnNode spawnNodeCall;
	spawnNodeCall.request.manufacturer = moduleProxy->getModuleIdentifier().getManufacturer();
	spawnNodeCall.request.typeNumber = moduleProxy->getModuleIdentifier().getTypeNumber();
	spawnNodeCall.request.serialNumber = moduleProxy->getModuleIdentifier().getSerialNumber();
	spawnNodeClient.waitForExistence();
	spawnNodeClient.call(spawnNodeCall);
}
void EquipletStateMachine::spawnModel(rexos_module::ModuleProxy* moduleProxy) {
	ROS_INFO_STREAM("Spawning model for " << moduleProxy->getModuleIdentifier());
	model_spawner_node::spawnModel spawnModelCall;
	spawnModelCall.request.manufacturer = moduleProxy->getModuleIdentifier().getManufacturer();
	spawnModelCall.request.typeNumber = moduleProxy->getModuleIdentifier().getTypeNumber();
	spawnModelCall.request.serialNumber = moduleProxy->getModuleIdentifier().getSerialNumber();
	spawnModelClient.waitForExistence();
	spawnModelClient.call(spawnModelCall);
}
void EquipletStateMachine::removeModel(rexos_module::ModuleProxy* moduleProxy) {
	ROS_INFO_STREAM("Removing model for " << moduleProxy->getModuleIdentifier());
	model_spawner_node::removeModel removeModelCall;
	removeModelCall.request.manufacturer = moduleProxy->getModuleIdentifier().getManufacturer();
	removeModelCall.request.typeNumber = moduleProxy->getModuleIdentifier().getTypeNumber();
	removeModelCall.request.serialNumber = moduleProxy->getModuleIdentifier().getSerialNumber();
	removeModelClient.waitForExistence();
	removeModelClient.call(removeModelCall);
}


bool EquipletStateMachine::allModulesInDesiredState(rexos_statemachine::State desiredState) {
	std::vector<rexos_module::ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	for(int i = 0; i < modules.size(); i++){
		if(modules[i]->getCurrentState() != desiredState){
			return false;
		}
	}
	return true;
}

bool EquipletStateMachine::transitionInitialize(){
	REXOS_INFO("transitionInitialize called");
	changeModuleStates(rexos_statemachine::STATE_SAFE);
	
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_SAFE)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	
	return true;
}
bool EquipletStateMachine::transitionDeinitialize(){
	REXOS_INFO("transitionDeinitialize called");
	changeModuleStates(rexos_statemachine::STATE_OFFLINE);
	
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_OFFLINE)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	
	return true;
}
bool EquipletStateMachine::transitionSetup(){
	REXOS_INFO("transitionSetup called");
	moduleRegistry.setNewRegistrationsAllowed(false);

	changeModuleStates(rexos_statemachine::STATE_STANDBY);
	
	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_STANDBY)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	
	return true;
}

bool EquipletStateMachine::transitionShutdown(){
	REXOS_INFO("transitionShutdown called");
	changeModuleStates(rexos_statemachine::STATE_SAFE);

	if(!allModulesInDesiredState(desiredState = rexos_statemachine::STATE_SAFE)) {
		boost::unique_lock<boost::mutex> lock( mutexit );
		condit.wait( lock );
	}
	moduleRegistry.setNewRegistrationsAllowed(true);
	return true;
}

bool EquipletStateMachine::transitionStart(){
	REXOS_INFO( "transitionStart called");
	return true;
}

bool EquipletStateMachine::transitionStop(){
	REXOS_INFO( "transitionStop called");
	return true;
}

std::vector<std::vector<rexos_datatypes::TransitionPhase>> EquipletStateMachine::calculateOrderOfCalibrationSteps() {
	std::vector<rexos_module::ModuleProxy*> modules = moduleRegistry.getRegisteredModules();
	std::vector<rexos_datatypes::TransitionPhase> pendingTransitionPhases;
	std::map<rexos_datatypes::SupportedMutation, std::vector<rexos_module::ModuleProxy*>> availiableSupportedMutations;
	for(int i = 0; i < modules.size(); i++) {
		rexos_knowledge_database::ModuleType moduleType = rexos_knowledge_database::ModuleType(modules.at(i)->getModuleIdentifier());
		std::vector<rexos_datatypes::TransitionPhase> transitionPhases =  moduleType.getTransitionPhases();
		
		// add all to the pendingTransitionPhases
		for(int j = 0; j < transitionPhases.size(); j++) {
			pendingTransitionPhases.push_back(transitionPhases.at(j));
			REXOS_INFO_STREAM(transitionPhases.at(j));
		}
	}
	
	std::vector<std::vector<rexos_datatypes::TransitionPhase>> output;
	// actually resolve the graph
	while(pendingTransitionPhases.size() != 0) {
		REXOS_INFO("New round");
		std::vector<rexos_datatypes::TransitionPhase> currentRound;
		
		std::vector<rexos_datatypes::TransitionPhase>::iterator it = pendingTransitionPhases.begin();
		while(it != pendingTransitionPhases.end()) {
			REXOS_INFO_STREAM("-" << *it);
			bool areAllRequiredMutationsAreAvailiable = areAllRequiredMutationsAvailiable(
					it->getRequiredMutations(), availiableSupportedMutations);
			REXOS_INFO_STREAM("-reqava " << areAllRequiredMutationsAreAvailiable);
			if(areAllRequiredMutationsAreAvailiable == true) {
				REXOS_INFO("a");
				// we can perform this calibrationStep
				currentRound.push_back(*it);
				REXOS_INFO("a");
				// we just removed an item from the list, but the erase gives an iterator to the next item
				it = pendingTransitionPhases.erase(it);
				REXOS_INFO("a");
			} else {
				REXOS_INFO("b");
				// just try the next one
				it++;
			}
		}
		REXOS_INFO("c");
		if(currentRound.size() == 0) {
			// there was a cyclic dependency (or a depencency is just not supported) as no new transition phase could be performed
			// add the remaining pendingTransitionPhases to the output in sequentional order
			for(int i = 0; i < pendingTransitionPhases.size(); i++) {
				std::vector<rexos_datatypes::TransitionPhase> transitionPhasesToAdd;
				transitionPhasesToAdd.push_back(pendingTransitionPhases.at(i));
				output.push_back(transitionPhasesToAdd);
			}
			throw CyclicDependencyException(output);
		} else {
			// we succesfully completed a round, add the gained transition phases to the availiable list
			for(int i = 0; i < currentRound.size(); i++) {
				std::vector<rexos_datatypes::SupportedMutation> supportedMutations = currentRound.at(i).getSupportedMutations();
				for(int j = 0; j < supportedMutations.size(); j++) {
					// we are not interested in the moduleProxies supporting the mutation, and thus do not store them
					availiableSupportedMutations[supportedMutations.at(j)];
				}
			}
		}
	}
	REXOS_INFO("done");
	return output;
}
bool EquipletStateMachine::areAllRequiredMutationsAvailiable(std::vector<rexos_datatypes::RequiredMutation> requiredMutations, 
		std::map<rexos_datatypes::SupportedMutation, std::vector<rexos_module::ModuleProxy*>> supportedMutations) {
	bool areAllRequiredMutationsAvailiable = true;
	for(int j = 0; j < requiredMutations.size(); j++) {
		rexos_datatypes::RequiredMutation requiredMutation = requiredMutations.at(j);
		
		bool isCurrentRequiredMutationsAvailiable = false;
		for(auto it = supportedMutations.begin(); it != supportedMutations.end(); it++) {
			rexos_datatypes::SupportedMutation supportedMutation = it->first;
			if(requiredMutation == supportedMutation) isCurrentRequiredMutationsAvailiable = true;
		}
		REXOS_INFO_STREAM("-reqcurrava " << isCurrentRequiredMutationsAvailiable);
		if(isCurrentRequiredMutationsAvailiable == false) {
			areAllRequiredMutationsAvailiable = false;
		}
	}
	return areAllRequiredMutationsAvailiable;
}
