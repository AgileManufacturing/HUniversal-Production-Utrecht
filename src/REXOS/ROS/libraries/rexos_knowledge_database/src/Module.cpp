#include <rexos_knowledge_database/Module.h>
#include <rexos_knowledge_database/KnowledgeDatabaseException.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

#include "ros/ros.h"

namespace rexos_knowledge_database{
	Module::Module(std::string manufacturer, std::string typeNumber, std::string serialNumber) :
				manufacturer(manufacturer), typeNumber(typeNumber), serialNumber(serialNumber)
	{
		connection = rexos_knowledge_database::connect();
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT * \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw KnowledgeDatabaseException("This module does not exist");
		}
		ROS_INFO_STREAM("Constructed module with manufacturer=" << manufacturer << 
				" typeNumber=" << typeNumber << " serialNumber=" << serialNumber);
	}
	ModuleType* Module::getModuleType(){
		return new rexos_knowledge_database::ModuleType(manufacturer, typeNumber);
	}
	std::string Module::getModuleProperties(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT moduleProperties \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string jsonProperties = result->getString("moduleProperties");
		
		delete preparedStmt;
		return jsonProperties;
	}
	void Module::setModuleProperties(std::string jsonProperties){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		UPDATE Module \
		SET moduleProperties = ? \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, jsonProperties);
		preparedStmt->setString(2, manufacturer);
		preparedStmt->setString(3, typeNumber);
		preparedStmt->setString(4, serialNumber);

		preparedStmt->executeQuery();
		delete preparedStmt;
	}
	Module* Module::getParentModule(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT attachedToManufacturer, attachedToTypeNumber, attachedToSerialNumber \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1) {
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		} else {
			// set the cursor at the first result
			result->next();
			if(result->isNull("attachedToManufacturer") || result->isNull("attachedToTypeNumber") || result->isNull("attachedToSerialNumber")) {
				return 0;
			} else {
				std::string attachedToManufacturer = result->getString("attachedToManufacturer");
				std::string attachedToTypeNumber = result->getString("attachedToTypeNumber");
				std::string attachedToSerialNumber = result->getString("attachedToSerialNumber");
				
				delete preparedStmt;
				return new Module(attachedToManufacturer, attachedToTypeNumber, attachedToSerialNumber);
			}
		}
	}
	std::vector<Module*> Module::getChildModules(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT manufacturer, typeNumber, serialNumber \
		FROM Module \
		WHERE attachedToManufacturer = ? AND \
		attachedToTypeNumber = ? AND \
		attachedToSerialNumber = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);

		sql::ResultSet* result = preparedStmt->executeQuery();
		std::vector<Module* > childModules;
		if(result->rowsCount() != 0){
			// get all the childs
			while(result->next()){
				childModules.push_back(new Module(result->getString("manufacturer"), result->getString("typeNumber"), result->getString("serialNumber")));
			}
		}
		delete preparedStmt;
		return childModules;
	}

	int Module::getMountPointX(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mountPointX \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointX = result->getInt("mountPointX");
		
		delete preparedStmt;
		return mountPointX;
	}
	void Module::setMountPointX(int mountPointX){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		UPDATE Module \
		SET mountPointY = ? \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setInt(1, mountPointX);
		preparedStmt->setString(2, manufacturer);
		preparedStmt->setString(3, typeNumber);
		preparedStmt->setString(4, serialNumber);
		
		preparedStmt->executeQuery();
		delete preparedStmt;
	}
	int Module::getMountPointY(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mountPointY \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointY = result->getInt("mountPointY");
		
		delete preparedStmt;
		return mountPointY;		
	}
	void Module::setMountPointY(int mountPointY){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		UPDATE Module \
		SET mountPointY = ? \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setInt(1, mountPointY);
		preparedStmt->setString(2, manufacturer);
		preparedStmt->setString(3, typeNumber);
		preparedStmt->setString(4, serialNumber);
		
		preparedStmt->executeQuery();
		delete preparedStmt;
	}

	std::string Module::getCalibrationDataForModuleOnly(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT properties \
		FROM ModuleCalibration \
		JOIN ModuleCalibrationGroup ON ModuleCalibrationGroup.ModuleCalibration = ModuleCalibration.id \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ? AND \
		( \
			SELECT count(*) \
			FROM ModuleCalibrationGroup AS subTable \
			WHERE ModuleCalibrationGroup.ModuleCalibration = subTable.ModuleCalibration \
		) = 1;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw KnowledgeDatabaseException("Unable to find calibration entry for only this module (there might be a calibration \
					entry shared with another module)");
		}
		// set the cursor at the first result
		result->next();
		std::string properties = result->getString("properties");
		delete preparedStmt;
		return properties;
	}
	std::string Module::getCalibrationDataForModuleAndChilds(){
		std::vector<Module*> childs = getChildModules();
		return getCalibrationDataForModuleAndOtherModules(childs);
	}
	std::string Module::getCalibrationDataForModuleAndOtherModules(std::vector<Module*> modules){
		int calibrationId = getCalibrationGroupForModuleAndOtherModules(modules);
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT properties \
		FROM ModuleCalibration \
		WHERE id = ?;");
		preparedStmt->setInt(1, calibrationId);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find calibration entry");
		}
		// set the cursor at the first result
		result->next();
		std::string properties = result->getString("properties");
		delete preparedStmt;
		return properties;
	}
	void Module::setCalibrationDataForModuleOnly(std::string properties){
		sql::PreparedStatement* preparedStmt;
		try{
			std::vector<Module*> emptyList;
			int calibrationId = getCalibrationGroupForModuleAndOtherModules(emptyList);
			
			// update existing entry
			preparedStmt = connection->prepareStatement("\
			UPDATE ModuleCalibration \
			SET properties = ? \
			WHERE id = ?;");
			preparedStmt->setString(1, properties);
			preparedStmt->setInt(2, calibrationId);
			preparedStmt->executeQuery();
			delete preparedStmt;
		} catch (KnowledgeDatabaseException ex) {
			// create a new entry
			preparedStmt = connection->prepareStatement("\
			INSERT INTO ModuleCalibration (properties) \
			VALUES (?);");
			preparedStmt->setString(1, properties);
			preparedStmt->executeQuery();
			delete preparedStmt;
			
			sql::PreparedStatement* preparedStmt;
			preparedStmt = connection->prepareStatement("\
			INSERT INTO ModuleCalibrationGroup (ModuleCalibration, manufacturer, typeNumber, serialNumber) \
			VALUES (LAST_INSERT_ID(), ?, ?, ?);");
			
			preparedStmt->setString(1, manufacturer);
			preparedStmt->setString(2, typeNumber);
			preparedStmt->setString(3, serialNumber);
			
			preparedStmt->executeQuery();
			delete preparedStmt;
		}
	}
	void Module::setCalibrationDataForModuleAndChilds(std::string properties){
		std::vector<Module*> childs = getChildModules();
		setCalibrationDataForModuleAndOtherModules(childs, properties);
	}
	void Module::setCalibrationDataForModuleAndOtherModules(std::vector<Module*> modules, std::string properties){
		sql::PreparedStatement* preparedStmt;
		try{
			int calibrationId = getCalibrationGroupForModuleAndOtherModules(modules);
			
			// update existing entry
			preparedStmt = connection->prepareStatement("\
			UPDATE ModuleCalibration \
			SET properties = ? \
			WHERE id = ?;");
			preparedStmt->setString(1, properties);
			preparedStmt->setInt(2, calibrationId);
			preparedStmt->executeQuery();
			delete preparedStmt;
		} catch (KnowledgeDatabaseException ex) {
			// create a new entry
			preparedStmt = connection->prepareStatement("\
			INSERT INTO ModuleCalibration (properties) \
			VALUES (?);");
			preparedStmt->setString(1, properties);
			preparedStmt->executeQuery();
			delete preparedStmt;
			
			sql::PreparedStatement* preparedStmt;
			preparedStmt = connection->prepareStatement("\
			INSERT INTO ModuleCalibrationGroup (ModuleCalibration, manufacturer, typeNumber, serialNumber) \
			VALUES (LAST_INSERT_ID(), ?, ?, ?);");
			
			preparedStmt->setString(1, manufacturer);
			preparedStmt->setString(2, typeNumber);
			preparedStmt->setString(3, serialNumber);
			preparedStmt->executeQuery();
			
			for(int i = 0; i < modules.size(); i++){
				preparedStmt->setString(1, modules.at(i)->manufacturer);
				preparedStmt->setString(2, modules.at(i)->typeNumber);
				preparedStmt->setString(3, modules.at(i)->serialNumber);
				preparedStmt->executeQuery();
			}
			delete preparedStmt;
		}
	}
	
	
	int Module::getCalibrationGroupForModuleAndOtherModules(std::vector<Module*> modules){
		// create a temp table for storing the modules
		sql::PreparedStatement* preparedStmt;
		preparedStmt = connection->prepareStatement("\
		CREATE TEMPORARY TABLE otherModules( \
			manufacturer char(200) NOT NULL, \
			typeNumber char(200) NOT NULL, \
			serialNumber char(200) NOT NULL \
		);");
		preparedStmt->executeQuery();
		delete preparedStmt;
		
		// store the modules
		preparedStmt = connection->prepareStatement("\
		INSERT INTO otherModules( \
			manufacturer, typeNumber, serialNumber \
		) VALUES ( \
			?, ?, ? \
		);");
		for(int i = 0; i < modules.size(); i++){
			preparedStmt->setString(1, modules.at(i)->manufacturer);
			preparedStmt->setString(2, modules.at(i)->typeNumber);
			preparedStmt->setString(3, modules.at(i)->serialNumber);
			preparedStmt->executeQuery();
		}
		delete preparedStmt;
		
		// preform the actual query
		preparedStmt = connection->prepareStatement("\n\
		SELECT id \n\
		FROM ModuleCalibration \n\
		JOIN ModuleCalibrationGroup ON ModuleCalibrationGroup.ModuleCalibration = ModuleCalibration.id \n\
		WHERE manufacturer = ? AND \n\
		typeNumber = ? AND \n\
		serialNumber = ? AND \n\
		( -- to narrow group. The number of modules in this group must match the number of other modules \n\
			SELECT count(*) \n\
			FROM ModuleCalibrationGroup AS inListGroup \n\
			JOIN otherModules ON \n\
				inListGroup.manufacturer = otherModules.manufacturer AND \n\
				inListGroup.typeNumber = otherModules.typeNumber AND \n\
				inListGroup.serialNumber = otherModules.serialNumber \n\
			WHERE ModuleCalibrationGroup.ModuleCalibration = inListGroup.ModuleCalibration \n\
		) = ? AND \n\
		( -- to wide group. The number of modules in this group must match the number of other modules \n\
			SELECT count(*) \n\
			FROM ModuleCalibrationGroup AS listGroup \n\
			WHERE ModuleCalibrationGroup.ModuleCalibration = listGroup.ModuleCalibration AND ( \n\
				listGroup.manufacturer != ModuleCalibrationGroup.manufacturer OR \n\
				listGroup.typeNumber != ModuleCalibrationGroup.typeNumber OR \n\
				listGroup.serialNumber != ModuleCalibrationGroup.serialNumber \n\
			) \n\
		) = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);
		preparedStmt->setInt(4, modules.size());
		preparedStmt->setInt(5, modules.size());

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			// delete the temp table for storing the modules
			preparedStmt = connection->prepareStatement("\
			DROP TEMPORARY TABLE otherModules;");
			preparedStmt->executeQuery();
			delete preparedStmt;
			throw KnowledgeDatabaseException("Unable to find calibration entry for only this module and other modules");
		}
		// set the cursor at the first result
		result->next();
		int calibrationId = result->getInt("id");
		delete preparedStmt;
		
		// delete the temp table for storing the modules
		preparedStmt = connection->prepareStatement("\
		DROP TEMPORARY TABLE otherModules;");
		preparedStmt->executeQuery();
		delete preparedStmt;
		
		return calibrationId;
	}
}