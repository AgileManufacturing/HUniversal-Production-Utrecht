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
	ROS_INFO("ModuleRegistry: New module %s %s %s registering", req.manufacturer.c_str(), req.typeNumber.c_str(), req.serialNumber.c_str());
	
	if(!newRegistrationsAllowed) {
		ROS_INFO("registration of new module not allowed");
		return false;
	}

	ModuleProxy* proxy = new ModuleProxy(
			equipletName,
			rexos_knowledge_database::ModuleIdentifier(req.manufacturer, req.typeNumber, req.serialNumber),
			this);
	registeredModules.push_back(proxy);

	ROS_INFO("registration successful");

	return true;
}

void ModuleRegistry::onModuleStateChanged(
	ModuleProxy* moduleProxy,
	rexos_statemachine::State newState, 
	rexos_statemachine::State previousState)
{
	//ROS_INFO("ModuleRegistry received from %s a state change from %s to %s",moduleProxy->getModuleNodeName(),previousState,newState);
	if(moduleRegistryListener != NULL){
		moduleRegistryListener->onModuleStateChanged(moduleProxy, newState, previousState);
	}
}

void ModuleRegistry::onModuleModeChanged(
	ModuleProxy* moduleProxy, 
	rexos_statemachine::Mode newMode, 
	rexos_statemachine::Mode previousMode)
{
	//ROS_INFO("ModuleRegistry received from %s a mode change from %s to %s",moduleProxy->getModuleNodeName(),previousMode,newMode);
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
	ROS_WARN("Module has died! :(");
	for(std::vector<ModuleProxy*>::iterator it = registeredModules.begin(); it != registeredModules.end(); it++){
		if(*it == moduleProxy){
			ROS_INFO("found me");
			registeredModules.erase(it);
			break;
		}
	}
}

} /* namespace equiplet_node */
