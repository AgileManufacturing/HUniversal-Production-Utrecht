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
#include <equiplet_node/ModuleRegistryListener.h>
#include <rexos_module/ModuleProxy.h>
#include <rexos_module/ModuleProxyListener.h>
#include <rexos_knowledge_database/Equiplet.h>
#include <rexos_datatypes/ModuleIdentifier.h>
#include <rexos_datatypes/RequiredMutation.h>
#include <rexos_datatypes/SupportedMutation.h>
#include <rexos_logger/rexos_logger.h>

namespace equiplet_node {

class EquipletNode;

class ModuleRegistry : public rexos_module::ModuleProxyListener {
public:
	ModuleRegistry(std::string equipletName, ModuleRegistryListener* mrl = NULL);
	virtual ~ModuleRegistry();

	void setModuleRegistryListener(ModuleRegistryListener* mrl);

	void setNewRegistrationsAllowed(bool allowed);

	void onModuleStateChanged(rexos_module::ModuleProxy* moduleProxy, rexos_statemachine::State newState, rexos_statemachine::State previousState);
	void onModuleModeChanged(rexos_module::ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode);
	void onHardwareStepCompleted(rexos_module::ModuleInterface* moduleInterface, std::string id, bool completed);
	void onModuleDied(rexos_module::ModuleProxy* moduleProxy);
	void onModuleTransitionPhaseCompleted(rexos_module::ModuleProxy* moduleProxy, 
			std::vector<rexos_datatypes::SupportedMutation> gainedSupportedMutations, 
			std::vector<rexos_datatypes::RequiredMutation> requiredMutationsRequiredForNextPhase);
	void spawnNode(rexos_module::ModuleProxy* moduleProxy);
	
	std::vector<rexos_module::ModuleProxy*> getRegisteredModules();

	rexos_module::ModuleProxy* getModule(rexos_datatypes::ModuleIdentifier moduleIdentifier);
	
	/**
	 * [reloadModules - WIP LARS]
	 */
 	void reloadModules();

private:
	bool onRegisterServiceModuleCallback(RegisterModule::Request &req, RegisterModule::Response &res);

	ModuleRegistryListener* moduleRegistryListener;

	ros::NodeHandle rosNodeHandle;
	ros::ServiceServer registerModuleServiceServer;

	bool newRegistrationsAllowed;
	std::string equipletName;

	std::vector<rexos_module::ModuleProxy*> registeredModules;
};

} /* namespace equiplet_node */
#endif /* MODULEREGISTRY_H_ */
