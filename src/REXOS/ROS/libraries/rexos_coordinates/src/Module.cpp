#include <rexos_coordinates/Module.h>

#include "ros/ros.h"

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <jsoncpp/json/value.h>
#include <jsoncpp/json/reader.h>

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
		
		std::string properties = module->getModuleTypeProperties();
		Json::Reader reader;
		Json::Value jsonNode;
		bool parseSuccesfull = reader.parse(properties, jsonNode);
		if(parseSuccesfull == false) throw std::runtime_error("module properties is not a valid json string");
		
		double midPointX = std::numeric_limits<double>::quiet_NaN();
		double midPointY = std::numeric_limits<double>::quiet_NaN();
		double midPointZ = std::numeric_limits<double>::quiet_NaN();
		ROS_INFO_STREAM(properties);
		//ROS_INFO_STREAM(jsonNode.write_formatted());
		midPointX = jsonNode["midPointX"].asDouble();
		midPointY = jsonNode["midPointY"].asDouble();
		midPointZ = jsonNode["midPointZ"].asDouble();
		
		if(std::isnan(midPointX) || std::isnan(midPointY) || std::isnan(midPointZ)){
			throw std::runtime_error("The properties do not contain the midPoints");
		}
		
		// TODO hardcoded?
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