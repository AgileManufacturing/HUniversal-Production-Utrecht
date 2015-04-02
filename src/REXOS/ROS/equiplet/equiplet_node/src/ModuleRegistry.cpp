/*
 * ModuleRegistry.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 *      Author: Lars Veenendaal
 */

#include <equiplet_node/ModuleRegistry.h>

#include <equiplet_node/EquipletNode.h>

namespace equiplet_node {

ModuleRegistry::ModuleRegistry(std::string equipletName, ModuleRegistryListener* mrl) :
		newRegistrationsAllowed(false),
		equipletName(equipletName),
		moduleRegistryListener(mrl)
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
		std::string x = equiplet.checkIfModuleStillExistInDatabase(proxy->getModuleIdentifier().getManufacturer().c_str(), proxy->getModuleIdentifier().getTypeNumber().c_str(), proxy->getModuleIdentifier().getSerialNumber().c_str());
	    if(x == "0"){
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
	for(auto it = registeredModules.begin(); it != registeredModules.end(); it++) {
		delete *it;
	}
}

void ModuleRegistry::setModuleRegistryListener(ModuleRegistryListener* mrl){
	moduleRegistryListener = mrl;
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
	for (int i = 0; i < registeredModules.size(); i++) {
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

void ModuleRegistry::onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, std::string id, bool completed) {
	if(moduleRegistryListener != NULL) {
		moduleRegistryListener->onHardwareStepCompleted(moduleInterface, id, completed);
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
