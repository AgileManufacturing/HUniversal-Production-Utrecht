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

namespace equiplet_node {

class EquipletNode;

class ModuleRegistry {
public:
	ModuleRegistry(std::string nodeName, int equipletId);
	virtual ~ModuleRegistry();

	void setNewRegistrationsAllowed(bool allowed);

	std::vector<ModuleProxy*> getRigisteredModules();
private:
	bool onRegisterServiceModuleCallback(RegisterModule::Request &req, RegisterModule::Response &res);

	ros::NodeHandle rosNodeHandle;
	ros::ServiceServer registerModuleServiceServer;

	bool newRegistrationsAllowed;
	int equipletId;

	std::vector<ModuleProxy*> registeredModules;
};

} /* namespace equiplet_node */
#endif /* MODULEREGISTRY_H_ */
