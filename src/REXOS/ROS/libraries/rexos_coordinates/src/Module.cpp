#include <rexos_coordinates/Module.h>

#include "ros/ros.h"

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <libjson/libjson.h>

#include <stdexcept>
#include <cmath>
#include <limits>

namespace rexos_coordinates{
	Vector3 Module::convertToEquipletCoordinate(Vector3 moduleCoordinate){
		return moduleCoordinate - moduleToEquiplet;
	}
	Vector3 Module::convertToModuleCoordinate(Vector3 equipletCoordinate){
		return equipletCoordinate - equipletToModule;
	}
	Module::Module(rexos_knowledge_database::Module* module) :
			module(module)
	{
		updateTranslationVectors();
		ROS_INFO_STREAM("rexos_coordinates: Constructed module");
	}
	void Module::updateTranslationVectors() {
		int mountPointX = module->getMountPointX();
		int mountPointY = module->getMountPointY();
		
		rexos_knowledge_database::ModuleType* moduleType = module->getModuleType();
		std::string properties = moduleType->getModuleTypeProperties();
		JSONNode jsonNode = libjson::parse(properties);
		
		double midPointX = std::numeric_limits<double>::quiet_NaN();
		double midPointY = std::numeric_limits<double>::quiet_NaN();
		double midPointZ = std::numeric_limits<double>::quiet_NaN();
		ROS_INFO_STREAM(properties);
		//ROS_INFO_STREAM(jsonNode.write_formatted());
		for(JSONNode::const_iterator it = jsonNode.begin(); it != jsonNode.end(); it++) {
			if(it->name() == "midPointX"){
				midPointX = it->as_float();
				ROS_INFO("found midPointX");
			} else if(it->name() == "midPointY"){
				midPointY = it->as_float();
				ROS_INFO("found midPointY");
			} else if(it->name() == "midPointZ"){
				midPointZ = it->as_float();
				ROS_INFO("found midPointZ");
			} else {
				// some other property, ignore it
			}
		}
		if(std::isnan(midPointX) || std::isnan(midPointY) || std::isnan(midPointZ)){
			throw std::runtime_error("The properties do not contain the midPoints");
		}
		
		double mountPointsDistanceX = 50;
		double mountPointsDistanceY = 50;
		
		double offsetX = mountPointX * mountPointsDistanceX + midPointX;
		double offsetY = midPointY;
		double offsetZ = -(mountPointY * mountPointsDistanceY) + midPointZ;
		
		this->equipletToModule = Vector3(offsetX, offsetY, offsetZ);
		this->moduleToEquiplet = Vector3(-offsetX, -offsetY, -offsetZ);
		ROS_INFO_STREAM("equipletToModule: " << equipletToModule);
		ROS_INFO_STREAM("moduleToEquiplet: " << moduleToEquiplet);
	}
}