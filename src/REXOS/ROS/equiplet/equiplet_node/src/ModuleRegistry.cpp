/*
 * ModuleRegistry.cpp
 *
 *  Created on: Jun 14, 2013
 *	  Author: joris
 *	  Author: Lars Veenendaal
 */

#include <equiplet_node/ModuleRegistry.h>

#include <equiplet_node/EquipletNode.h>

namespace equiplet_node {

ModuleRegistry::ModuleRegistry(std::string equipletName, ModuleRegistryListener* mrl) :
		moduleRegistryListener(mrl),
		newRegistrationsAllowed(false),
		equipletName(equipletName)
{
	registerModuleServiceServer = rosNodeHandle.advertiseService(
			equipletName + "/register_module",
			&ModuleRegistry::onRegisterServiceModuleCallback,
			this);
	rexos_knowledge_database::Equiplet equiplet = rexos_knowledge_database::Equiplet(equipletName);
	std::vector<rexos_datatypes::ModuleIdentifier> identifiers = equiplet.getModuleIdentifiersOfAttachedModulesWithRosSoftware();
	
	for(auto it = identifiers.begin(); it < identifiers.end(); it++) {
		rexos_module::ModuleProxy* proxy = new rexos_module::ModuleProxy(equipletName, *it, this);
		registeredModules.push_back(proxy);
	}
}
void ModuleRegistry::spawnModels() {
	rexos_knowledge_database::Equiplet equiplet = rexos_knowledge_database::Equiplet(equipletName);
	std::vector<rexos_datatypes::ModuleIdentifier> identifiers = equiplet.getModuleIdentifiersOfAttachedModules();
	
	// we must spawn the modules in a specifiec order (from top to bottom)
	std::vector<rexos_datatypes::ModuleIdentifier> processedIdentifiers;
	
	// first spawn all the top modules
	for(auto it = identifiers.begin(); it < identifiers.end(); it++) {
		auto databaseEntry = rexos_knowledge_database::Module(*it);
		if(databaseEntry.getParentModule() == NULL) {
			moduleRegistryListener->spawnModel(*it);
			processedIdentifiers.push_back(*it);
		}
	}
	
	// continue spawning modules if the parent has already been spawened
	while(processedIdentifiers.size() < identifiers.size()) {
		uint numberOfProcessedModulesBeforeRound = processedIdentifiers.size();
		
		for(auto it = identifiers.begin(); it < identifiers.end(); it++) {
			rexos_knowledge_database::Module databaseEntry(*it);
			// has this module already a model?
			if(std::find(processedIdentifiers.begin(), processedIdentifiers.end(), *it) == processedIdentifiers.end()) {
				// no, determine the parent of this module
				rexos_knowledge_database::Module* parentModuleDatabaseEntry = databaseEntry.getParentModule();
				rexos_module::ModuleProxy* parentModule = getModule(parentModuleDatabaseEntry->getModuleIdentifier());
				
				// has the parent of this module already a model?
				if(std::find(processedIdentifiers.begin(), processedIdentifiers.end(), parentModule->getModuleIdentifier()) != processedIdentifiers.end()) {
					// yes, and thus we can spawn it
					moduleRegistryListener->spawnModel(*it);
					processedIdentifiers.push_back(*it);
				}
			}
		}
		
		if(numberOfProcessedModulesBeforeRound == processedIdentifiers.size()) {
			throw std::runtime_error("Unable to spawn models for all the modules, is a module orphan?");
		}
	}
}

void ModuleRegistry::reloadModules(){
	// 
	// Checks if a new modules is actually in the knowledge database. 
	// If then register the new found modules.
	// 
	
	rexos_knowledge_database::Equiplet equiplet = rexos_knowledge_database::Equiplet(equipletName);
	std::vector<rexos_datatypes::ModuleIdentifier> identifiers = equiplet.getModuleIdentifiersOfAttachedModulesWithRosSoftware();
	for(auto it = identifiers.begin(); it < identifiers.end(); it++) {

		if(this->getModule(*it) == NULL){
			// ADD new module if not already added.
			REXOS_INFO("Registering a new module.");
			rexos_module::ModuleProxy* proxy = new rexos_module::ModuleProxy(equipletName, *it, this);
			registeredModules.push_back(proxy);
			proxy->changeState(rexos_statemachine::State::STATE_SAFE);
		}
	}

	// 
	// Check wether there is a registered module that is not known in the database.
	// Then shutdown the modules thats not found in the knowledge database.
	// And delete them and break there bond.
	// 
	auto track = registeredModules.begin();
	while (track != registeredModules.end()) {
		rexos_module::ModuleProxy* proxy = *track;
		bool isInDatabase = equiplet.checkIfModuleStillExistInDatabase(proxy->getModuleIdentifier());
		if(isInDatabase == false){
			REXOS_INFO("Removing a old module.");
			proxy->changeState(rexos_statemachine::State::STATE_OFFLINE);
			delete proxy;
			track = registeredModules.erase(track);
		}
		else {
			++track;
		}
	}
	REXOS_INFO("reloadModules() Succesfull.");
}

ModuleRegistry::~ModuleRegistry() {
	rexos_knowledge_database::Equiplet equiplet = rexos_knowledge_database::Equiplet(equipletName);
	std::vector<rexos_datatypes::ModuleIdentifier> identifiers = equiplet.getModuleIdentifiersOfAttachedModulesWithRosSoftware();
	for(auto it = identifiers.begin(); it < identifiers.end(); it++) {
		moduleRegistryListener->removeModel(*it);
	}
	for(auto it = registeredModules.begin(); it != registeredModules.end(); it++) {
		delete *it;
	}
}

void ModuleRegistry::setNewRegistrationsAllowed(bool allowed){
	newRegistrationsAllowed = allowed;
}

std::vector<rexos_module::ModuleProxy*> ModuleRegistry::getRegisteredModules(){
	return registeredModules;
}

rexos_module::ModuleProxy* ModuleRegistry::getModule(rexos_datatypes::ModuleIdentifier moduleIdentifier){
	for(rexos_module::ModuleProxy* proxy : registeredModules){
		if(proxy->getModuleIdentifier() == moduleIdentifier)
			return proxy;
	}
	return NULL;
}

bool ModuleRegistry::onRegisterServiceModuleCallback(
	RegisterModule::Request &req, 
	RegisterModule::Response &res) 
{
	REXOS_INFO("ModuleRegistry: Module %s %s %s registering", req.manufacturer.c_str(), req.typeNumber.c_str(), req.serialNumber.c_str());
	
	if(!newRegistrationsAllowed) {
		REXOS_WARN("registration of new module not allowed");
		return false;
	}
	
	rexos_datatypes::ModuleIdentifier newModuleIdentifier(req.manufacturer, req.typeNumber, req.serialNumber);
	for (uint i = 0; i < registeredModules.size(); i++) {
		if(registeredModules[i]->getModuleIdentifier() == newModuleIdentifier) {
			registeredModules[i]->bind();
			REXOS_INFO("registration successful");
			return true;
		}
	}
	REXOS_WARN("registration failed because no module with this identifier exists in registry (did someone manually start a module with the wrong equipletName?)");
	return false;
}

void ModuleRegistry::onModuleStateChanged(
	rexos_module::ModuleProxy* moduleProxy,
	rexos_statemachine::State newState, 
	rexos_statemachine::State previousState)
{
	//REXOS_INFO("ModuleRegistry received from %s a state change from %s to %s",moduleProxy->getModuleNodeName(),previousState,newState);
	if(moduleRegistryListener != NULL){
		moduleRegistryListener->onModuleStateChanged(moduleProxy, newState, previousState);
	}
}

void ModuleRegistry::onModuleModeChanged(
	rexos_module::ModuleProxy* moduleProxy, 
	rexos_statemachine::Mode newMode, 
	rexos_statemachine::Mode previousMode)
{
	//REXOS_INFO("ModuleRegistry received from %s a mode change from %s to %s",moduleProxy->getModuleNodeName(),previousMode,newMode);
	if(moduleRegistryListener != NULL){
		moduleRegistryListener->onModuleModeChanged(moduleProxy, newMode, previousMode);
	}
}

void ModuleRegistry::onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, rexos_datatypes::HardwareStep hardwareStep) {
	if(moduleRegistryListener != NULL) {
		moduleRegistryListener->onHardwareStepCompleted(moduleInterface, hardwareStep);
	}
}

void ModuleRegistry::onModuleDied(rexos_module::ModuleProxy* moduleProxy){
	if(moduleRegistryListener != NULL) {
		moduleRegistryListener->onModuleDied(moduleProxy);
	}
}

void ModuleRegistry::onModuleTransitionPhaseCompleted(
	rexos_module::ModuleProxy* moduleProxy, 
	std::vector<rexos_datatypes::SupportedMutation> gainedSupportedMutations, 
	std::vector<rexos_datatypes::RequiredMutation> requiredMutationsRequiredForNextPhase) 
{
	moduleRegistryListener->onModuleTransitionPhaseCompleted(moduleProxy, gainedSupportedMutations, requiredMutationsRequiredForNextPhase);
}
void ModuleRegistry::spawnNode(rexos_module::ModuleProxy* moduleProxy) {
	moduleRegistryListener->spawnNode(moduleProxy);
}
} /* namespace equiplet_node */
