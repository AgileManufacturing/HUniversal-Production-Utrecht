#include <rexos_coordinates/Module.h>

#include "ros/ros.h"

#include <rexos_knowledge_database/rexos_knowledge_database.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <stdexcept>

namespace rexos_coordinates{
	Vector3 Module::convertToEquipletCoordinate(Vector3 moduleCoordinate){
		return moduleCoordinate - moduleToEquiplet;
	}
	Vector3 Module::convertToModuleCoordinate(Vector3 equipletCoordinate){
		return equipletCoordinate - equipletToModule;
	}
	Module::Module(int moduleId) :
			moduleId(moduleId)
	{
		updateTranslationVectors();
	}
	void Module::updateTranslationVectors(){
		std::auto_ptr<sql::Connection> apConnection = rexos_knowledge_database::connect();
		
		// stmt = statement
		// pstmt = prepared statement
		sql::PreparedStatement* modulePStmt = apConnection->prepareStatement("\
		SELECT location, mountPointX, mountPointY, module_type \
		FROM modules \
		WHERE id = ?;");
		modulePStmt->setInt(1, moduleId);
		
		sql::ResultSet* moduleResult = modulePStmt->executeQuery();
		if(moduleResult->rowsCount() != 1){
			throw std::invalid_argument("rexos_coordinates::Module Module does not exist in database");
		} else {
			// set the cursor at the first result
			moduleResult->next(); 
			int equipletId = moduleResult->getInt("location");
			
			sql::PreparedStatement* equipletPStmt = apConnection->prepareStatement("\
			SELECT mountPoints_distanceX, mountPoints_distanceY \
			FROM equiplets \
			WHERE id = ?;");
			equipletPStmt->setInt(1, equipletId);
			
			sql::ResultSet* equipletResult = equipletPStmt->executeQuery();
			// set the cursor at the first result
			equipletResult->next(); 
			
			sql::PreparedStatement* moduleTypePStmt = apConnection->prepareStatement("\
			SELECT midPoint_x, midPoint_y, midPoint_z \
			FROM module_types \
			WHERE id = ?;");
			moduleTypePStmt->setInt(1, moduleResult->getInt("module_type"));
			
			sql::ResultSet* moduleTypeResult = moduleTypePStmt->executeQuery();
			// set the cursor at the first result
			moduleTypeResult->next(); 
			
			double offsetX = 
			(
				moduleResult->getDouble("mountPointX") * equipletResult->getDouble("mountPoints_distanceX")
			) + moduleTypeResult->getDouble("midPoint_x");
			double offsetY = moduleTypeResult->getDouble("midPoint_Y");
			double offsetZ = 
			(
				-(moduleResult->getDouble("mountPointY") * equipletResult->getDouble("mountPoints_distanceY"))
			) + moduleTypeResult->getDouble("midPoint_Z");
			//double offsetZ = moduleTypeResult->getDouble("midPoint_Z");
			
			this->equipletToModule = Vector3(offsetX, offsetY, offsetZ);
			this->moduleToEquiplet = Vector3(-offsetX, -offsetY, -offsetZ);
			ROS_INFO_STREAM("equipletToModule: " << equipletToModule);
			ROS_INFO_STREAM("moduleToEquiplet: " << moduleToEquiplet);
		}
	}
}