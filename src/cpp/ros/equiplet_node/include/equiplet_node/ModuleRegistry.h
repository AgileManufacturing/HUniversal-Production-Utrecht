/*
 * ModuleRegistry.h
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#ifndef MODULEREGISTRY_H_
#define MODULEREGISTRY_H_

#include <ros/ros.h>

#include <equiplet_node/RegisterModule.h>

#include <equiplet_node/ModuleProxy.h>

#include "equiplet_node/ModuleRegistryListener.h"

namespace equiplet_node {

class EquipletNode;

class ModuleRegistry : public equiplet_node::ModuleProxyListener {
public:
	ModuleRegistry(std::string nodeName, int equipletId, ModuleRegistryListener* mrl = NULL);
	virtual ~ModuleRegistry();

	void setModuleRegistryListener(ModuleRegistryListener* mrl);

	void setNewRegistrationsAllowed(bool allowed);

	void onModuleStateChanged(ModuleProxy* moduleProxy,rexos_statemachine::State newState, rexos_statemachine::State previousState);

	void onModuleModeChanged(ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode);

	void onInstructionStepCompleted(ModuleProxy* moduleProxy, std::string id, bool completed);

	std::vector<ModuleProxy*> getRegisteredModules();

	ModuleProxy* getModule(int moduleId);

private:
	bool onRegisterServiceModuleCallback(RegisterModule::Request &req, RegisterModule::Response &res);

	ModuleRegistryListener* moduleRegistryListener;

	ros::NodeHandle rosNodeHandle;
	ros::ServiceServer registerModuleServiceServer;

	bool newRegistrationsAllowed;
	int equipletId;

	std::vector<ModuleProxy*> registeredModules;
};

} /* namespace equiplet_node */
#endif /* MODULEREGISTRY_H_ */
