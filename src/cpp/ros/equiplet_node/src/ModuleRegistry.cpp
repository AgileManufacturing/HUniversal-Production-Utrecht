/*
 * ModuleRegistry.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#include <equiplet_node/ModuleRegistry.h>

#include <equiplet_node/EquipletNode.h>

namespace equiplet_node {

ModuleRegistry::ModuleRegistry(std::string nodeName, int equipletId, ModuleRegistryListener* mrl)
:newRegistrationsAllowed(false),equipletId(equipletId),moduleRegistryListener(mrl)
{
	registerModuleServiceServer = rosNodeHandle.advertiseService(
			nodeName + "/register_module",
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

std::vector<ModuleProxy*> ModuleRegistry::getRigisteredModules(){
	return registeredModules;
}

ModuleProxy* ModuleRegistry::getModule(int moduleId){
	for(ModuleProxy* proxy : registeredModules){
		if(proxy->getModuleId() == moduleId)
			return proxy;
	}
	return NULL;
}

bool ModuleRegistry::onRegisterServiceModuleCallback(RegisterModule::Request &req, RegisterModule::Response &res) {
	ROS_INFO("ModuleRegistry: New module %s with id %d registering", req.name.c_str(), req.id);
	if(!newRegistrationsAllowed){
		ROS_INFO("registration of new module not allowed");
		return false;
	}

	ModuleProxy* proxy = new ModuleProxy(
			EquipletNode::nameFromId(equipletId),
			req.name,
			equipletId,
			req.id,this);
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

} /* namespace equiplet_node */