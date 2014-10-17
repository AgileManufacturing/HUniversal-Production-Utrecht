/*
 * ModuleRegistry.h
 *
 *  Created on: Jun 14, 2013
 *      Author: joris
 */

#ifndef MODULEREGISTRY_H_
#define MODULEREGISTRY_H_

#include <ros/ros.h>
#include "rexos_logger/rexos_logger.h"

#include <equiplet_node/RegisterModule.h>

#include <equiplet_node/ModuleProxy.h>

#include "equiplet_node/ModuleRegistryListener.h"

#include <rexos_knowledge_database/ModuleIdentifier.h>
#include <rexos_knowledge_database/Equiplet.h>
#include <rexos_knowledge_database/RequiredMutation.h>
#include <rexos_knowledge_database/SupportedMutation.h>

namespace equiplet_node {

class EquipletNode;

class ModuleRegistry : public equiplet_node::ModuleProxyListener {
public:
	ModuleRegistry(std::string equipletName, ModuleRegistryListener* mrl = NULL);
	virtual ~ModuleRegistry();

	void setModuleRegistryListener(ModuleRegistryListener* mrl);

	void setNewRegistrationsAllowed(bool allowed);

	void onModuleStateChanged(ModuleProxy* moduleProxy,rexos_statemachine::State newState, rexos_statemachine::State previousState);

	void onModuleModeChanged(ModuleProxy* moduleProxy, rexos_statemachine::Mode newMode, rexos_statemachine::Mode previousMode);

	void onHardwareStepCompleted(ModuleProxy* moduleProxy, std::string id, bool completed);
	
	void onModuleDied(ModuleProxy* moduleProxy);
	
	void onModuleTransitionPhaseCompleted(ModuleProxy* moduleProxy, 
			std::vector<rexos_knowledge_database::SupportedMutation> gainedSupportedMutations, 
			std::vector<rexos_knowledge_database::RequiredMutation> requiredMutationsRequiredForNextPhase);

	std::vector<ModuleProxy*> getRegisteredModules();

	ModuleProxy* getModule(rexos_knowledge_database::ModuleIdentifier moduleIdentifier);

private:
	bool onRegisterServiceModuleCallback(RegisterModule::Request &req, RegisterModule::Response &res);

	ModuleRegistryListener* moduleRegistryListener;

	ros::NodeHandle rosNodeHandle;
	ros::ServiceServer registerModuleServiceServer;

	bool newRegistrationsAllowed;
	std::string equipletName;

	std::vector<ModuleProxy*> registeredModules;
};

} /* namespace equiplet_node */
#endif /* MODULEREGISTRY_H_ */
