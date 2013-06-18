/*
 * ModuleRegistry.cpp
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#include "equiplet_node/ModuleRegistry.h"

#include <equiplet_node/EquipletNode.h>

namespace equiplet_node {

ModuleRegistry::ModuleRegistry(std::string nodeName, int equipletId)
:newRegistrationsAllowed(false),
 equipletId(equipletId)
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

void ModuleRegistry::setNewRegistrationsAllowed(bool allowed){
	newRegistrationsAllowed = allowed;
}

std::vector<ModuleProxy*> ModuleRegistry::getRigisteredModules(){
	return registeredModules;
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
			req.id);
	registeredModules.push_back(proxy);

	ROS_INFO("registration successful");

	return true;
}

} /* namespace equiplet_node */
