/*
 * ModuleRegistry.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
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
	std::vector<rexos_knowledge_database::ModuleIdentifier> identifiers = equiplet.getModuleIdentifiersOfAttachedModulesWithRosSoftware();
	
	for(std::vector<rexos_knowledge_database::ModuleIdentifier>::iterator it = identifiers.begin(); it < identifiers.end(); it++) {
		ModuleProxy* proxy = new ModuleProxy(
				equipletName,
				rexos_knowledge_database::ModuleIdentifier(it->getManufacturer(), it->getTypeNumber(), it->getSerialNumber()),
				this);
		registeredModules.push_back(proxy);
	}
	
}

ModuleRegistry::~ModuleRegistry() {
	for(auto it = registeredModules.begin(); it != registeredModules.end(); it++) {
		ModuleProxy* proxy = *it;
		delete proxy;
	}
}

void ModuleRegistry::setModuleRegistryListener(ModuleRegistryListener* mrl){
	moduleRegistryListener = mrl;
}


void ModuleRegistry::setNewRegistrationsAllowed(bool allowed){
	newRegistrationsAllowed = allowed;
}

std::vector<ModuleProxy*> ModuleRegistry::getRegisteredModules(){
	return registeredModules;
}

ModuleProxy* ModuleRegistry::getModule(rexos_knowledge_database::ModuleIdentifier moduleIdentifier){
	for(ModuleProxy* proxy : registeredModules){
		if(proxy->getModuleIdentifier() == moduleIdentifier)
			return proxy;
	}
	return NULL;
}

bool ModuleRegistry::onRegisterServiceModuleCallback(RegisterModule::Request &req, RegisterModule::Response &res) {
	REXOS_INFO("ModuleRegistry: Module %s %s %s registering", req.manufacturer.c_str(), req.typeNumber.c_str(), req.serialNumber.c_str());
	
	if(!newRegistrationsAllowed) {
		REXOS_INFO("registration of new module not allowed");
		return false;
	}
	
	rexos_knowledge_database::ModuleIdentifier newModuleIdentifier(req.manufacturer, req.typeNumber, req.serialNumber);
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
	ModuleProxy* moduleProxy,
	rexos_statemachine::State newState, 
	rexos_statemachine::State previousState)
{
	//REXOS_INFO("ModuleRegistry received from %s a state change from %s to %s",moduleProxy->getModuleNodeName(),previousState,newState);
	if(moduleRegistryListener != NULL){
		moduleRegistryListener->onModuleStateChanged(moduleProxy, newState, previousState);
	}
}

void ModuleRegistry::onModuleModeChanged(
	ModuleProxy* moduleProxy, 
	rexos_statemachine::Mode newMode, 
	rexos_statemachine::Mode previousMode)
{
	//REXOS_INFO("ModuleRegistry received from %s a mode change from %s to %s",moduleProxy->getModuleNodeName(),previousMode,newMode);
	if(moduleRegistryListener != NULL){
		moduleRegistryListener->onModuleModeChanged(moduleProxy, newMode, previousMode);
	}
}

void ModuleRegistry::onInstructionStepCompleted(
	ModuleProxy* moduleProxy,
	std::string id, 
	bool completed){
	
	if(moduleRegistryListener != NULL){
		moduleRegistryListener->onInstructionStepCompleted(moduleProxy, id, completed);
	}
}

void ModuleRegistry::onModuleDied(ModuleProxy* moduleProxy){
	REXOS_WARN("Module has died! :(");
	for(std::vector<ModuleProxy*>::iterator it = registeredModules.begin(); it != registeredModules.end(); it++){
		if(*it == moduleProxy){
			REXOS_INFO("found me");
			registeredModules.erase(it);
			delete *it;
			break;
		}
	}
}

	void ModuleRegistry::onModuleTransitionPhaseCompleted(ModuleProxy* moduleProxy, 
			std::vector<rexos_knowledge_database::SupportedMutation> gainedSupportedMutations, 
			std::vector<rexos_knowledge_database::RequiredMutation> requiredMutationsRequiredForNextPhase) {
		moduleRegistryListener->onModuleTransitionPhaseCompleted(moduleProxy, gainedSupportedMutations, requiredMutationsRequiredForNextPhase);
	}
} /* namespace equiplet_node */
