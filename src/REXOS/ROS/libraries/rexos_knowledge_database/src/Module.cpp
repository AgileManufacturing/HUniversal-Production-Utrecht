#include <rexos_knowledge_database/Module.h>
#include <rexos_knowledge_database/ModuleIdentifier.h>
#include <rexos_knowledge_database/KnowledgeDatabaseException.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

#include "ros/ros.h"

namespace rexos_knowledge_database{
	Module::Module(ModuleIdentifier moduleIdentifier) :
			ModuleType(moduleIdentifier), 
			moduleIdentifier(moduleIdentifier)
	{
		connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT * \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			std::string message = "This module (" + moduleIdentifier.toString() + ") does not exist";
			throw KnowledgeDatabaseException(message.c_str());
		}
		REXOS_INFO_STREAM("Constructed module with manufacturer=" << moduleIdentifier.getManufacturer() << 
				" typeNumber=" << moduleIdentifier.getTypeNumber() << " serialNumber=" << moduleIdentifier.getSerialNumber());
		
		delete result;
	}
	ModuleIdentifier Module::getModuleIdentifier() {
		return moduleIdentifier;
	}
	std::string Module::getModuleProperties(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT moduleProperties \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;");
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string jsonProperties = result->getString("moduleProperties");
		
		delete result;
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
		preparedStmt->setString(2, moduleIdentifier.getManufacturer());
		preparedStmt->setString(3, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(4, moduleIdentifier.getSerialNumber());

		preparedStmt->executeQuery();
		delete preparedStmt;
	}
	Module* Module::getParentModule(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT manufacturer, typeNumber, serialNumber \
		FROM Module \
		WHERE attachedToLeft < (\
			SELECT attachedToLeft FROM Module \
			WHERE manufacturer = ? \
				AND typeNumber = ? \
				AND serialNumber = ? \
		) AND attachedToRight > (\
	 		SELECT attachedToRight FROM Module \
			WHERE manufacturer = ? \
				AND typeNumber = ? \
				AND serialNumber = ? \
		)\
		ORDER BY abs(attachedToLeft - attachedToRight) \
		ASC LIMIT 1;");
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		preparedStmt->setString(4, moduleIdentifier.getManufacturer());
		preparedStmt->setString(5, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(6, moduleIdentifier.getSerialNumber());

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1) {
			delete result;
			delete preparedStmt;
			return 0;
		} else {
			// set the cursor at the first result
			result->next();
			ModuleIdentifier identifier = ModuleIdentifier(
				result->getString("manufacturer"),
				result->getString("typeNumber"),
				result->getString("serialNumber")
			);
			delete result;
			delete preparedStmt;
			return new Module(identifier);
		}
	}
	std::vector<ModuleIdentifier> Module::getChildModulesIdentifiers(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT manufacturer, typeNumber, serialNumber \
		FROM Module \
		WHERE attachedToLeft > (\
			SELECT attachedToLeft FROM Module \
			WHERE manufacturer = ? \
				AND typeNumber = ? \
				AND serialNumber = ? \
		) AND attachedToRight < (\
	 		SELECT attachedToRight FROM Module \
			WHERE manufacturer = ? \
				AND typeNumber = ? \
				AND serialNumber = ? \
		);");
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		preparedStmt->setString(4, moduleIdentifier.getManufacturer());
		preparedStmt->setString(5, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(6, moduleIdentifier.getSerialNumber());

		sql::ResultSet* result = preparedStmt->executeQuery();
		std::vector<ModuleIdentifier> childModules;
		if(result->rowsCount() != 0){
			// get all the childs
			while(result->next()){
				ModuleIdentifier identifier = ModuleIdentifier(
					result->getString("manufacturer"),
					result->getString("typeNumber"),
					result->getString("serialNumber")
				);
				childModules.push_back(identifier);
			}
		}
		delete result;
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
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointX = result->getInt("mountPointX");
		
		delete result;
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
		preparedStmt->setString(2, moduleIdentifier.getManufacturer());
		preparedStmt->setString(3, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(4, moduleIdentifier.getSerialNumber());
		
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
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointY = result->getInt("mountPointY");
		
		delete result;
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
		preparedStmt->setString(2, moduleIdentifier.getManufacturer());
		preparedStmt->setString(3, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(4, moduleIdentifier.getSerialNumber());
		
		preparedStmt->executeQuery();
		delete preparedStmt;
	}

	std::string Module::getCalibrationDataForModuleOnly(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT properties \
		FROM ModuleCalibration \
		JOIN ModuleCalibrationModuleSet ON ModuleCalibrationModuleSet.ModuleCalibration = ModuleCalibration.id \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ? AND \
		( \
			SELECT count(*) \
			FROM ModuleCalibrationModuleSet AS subTable \
			WHERE ModuleCalibrationModuleSet.ModuleCalibration = subTable.ModuleCalibration \
		) = 1;");
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw KnowledgeDatabaseException("Unable to find calibration entry for only this module (there might be a calibration \
					entry shared with another module)");
		}
		// set the cursor at the first result
		result->next();
		std::string properties = result->getString("properties");
		delete result;
		delete preparedStmt;
		return properties;
	}
	std::string Module::getCalibrationDataForModuleAndChilds(){
		REXOS_INFO("getCalibrationDataForModuleAndChilds a1");
		std::vector<ModuleIdentifier> childs = getChildModulesIdentifiers();
		REXOS_INFO("getCalibrationDataForModuleAndChilds a2, vector size = %lu", childs.size());
		std::string returnValue = getCalibrationDataForModuleAndOtherModules(childs);
		REXOS_INFO("%s", returnValue.c_str());
		return returnValue;
	}
	std::string Module::getCalibrationDataForModuleAndOtherModules(std::vector<ModuleIdentifier> moduleIdentifiers){
		REXOS_INFO("getCalibrationDataForModuleAndOtherModules b1" );
		int calibrationId = getCalibrationGroupForModuleAndOtherModules(moduleIdentifiers);
		std::string query = "SELECT properties FROM ModuleCalibration WHERE id = ?;";
		REXOS_INFO("getCalibrationDataForModuleAndOtherModules b2, SQL query = %s", query.c_str());
		sql::PreparedStatement* preparedStmt = connection->prepareStatement(query);
		preparedStmt->setInt(1, calibrationId);
		REXOS_INFO("getCalibrationDataForModuleAndOtherModules b3, SQL preparedStatement = ");
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find calibration entry");
		}
		// set the cursor at the first result
		result->next();
		std::string properties = result->getString("properties");
		delete result;
		delete preparedStmt;
		return properties;
	}
	void Module::setCalibrationDataForModuleOnly(std::string properties){
		sql::PreparedStatement* preparedStmt;
		try{
			std::vector<ModuleIdentifier> emptyList;
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
			INSERT INTO ModuleCalibrationModuleSet (ModuleCalibration, manufacturer, typeNumber, serialNumber) \
			VALUES (LAST_INSERT_ID(), ?, ?, ?);");
			
			preparedStmt->setString(1, moduleIdentifier.getManufacturer());
			preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
			preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
			
			preparedStmt->executeQuery();
			delete preparedStmt;
		}
	}
	void Module::setCalibrationDataForModuleAndChilds(std::string properties){
		std::vector<ModuleIdentifier> childs = getChildModulesIdentifiers();
		setCalibrationDataForModuleAndOtherModules(childs, properties);
	}
	void Module::setCalibrationDataForModuleAndOtherModules(std::vector<ModuleIdentifier> moduleIdentifiers, std::string properties){
		sql::PreparedStatement* preparedStmt;
		try{
			int calibrationId = getCalibrationGroupForModuleAndOtherModules(moduleIdentifiers);
			
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
			INSERT INTO ModuleCalibrationModuleSet (ModuleCalibration, manufacturer, typeNumber, serialNumber) \
			VALUES (LAST_INSERT_ID(), ?, ?, ?);");
			
			preparedStmt->setString(1, moduleIdentifier.getManufacturer());
			preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
			preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
			preparedStmt->executeQuery();
			
			for(int i = 0; i < moduleIdentifiers.size(); i++){
				preparedStmt->setString(1, moduleIdentifiers.at(i).getManufacturer());
				preparedStmt->setString(2, moduleIdentifiers.at(i).getTypeNumber());
				preparedStmt->setString(3, moduleIdentifiers.at(i).getSerialNumber());
				preparedStmt->executeQuery();
			}
			delete preparedStmt;
		}
	}
	
	
	int Module::getCalibrationGroupForModuleAndOtherModules(std::vector<ModuleIdentifier> moduleIdentifiers){
		// create a temp table for storing the modules
		REXOS_INFO("getCalibrationGroupForModuleAndOtherModules c1");
		sql::PreparedStatement* preparedStmt;
		std::string query = "\
		CREATE TEMPORARY TABLE otherModules( \
			manufacturer char(200) NOT NULL, \
			typeNumber char(200) NOT NULL, \
			serialNumber char(200) NOT NULL \
		);";
		REXOS_INFO("%s", query.c_str());
		preparedStmt = connection->prepareStatement(query);
		preparedStmt->executeQuery();
		delete preparedStmt;
		
		// store the modules
		query = "\
		INSERT INTO otherModules( \
			manufacturer, typeNumber, serialNumber \
		) VALUES ( \
			?, ?, ? \
		);";
		REXOS_INFO("%s", query.c_str());
		preparedStmt = connection->prepareStatement(query);
		for(int i = 0; i < moduleIdentifiers.size(); i++){
				preparedStmt->setString(1, moduleIdentifiers.at(i).getManufacturer());
				preparedStmt->setString(2, moduleIdentifiers.at(i).getTypeNumber());
				preparedStmt->setString(3, moduleIdentifiers.at(i).getSerialNumber());
			preparedStmt->executeQuery();
		}
		delete preparedStmt;
		
		// preform the actual query
		query = "\
		SELECT id \
		FROM ModuleCalibration \
		JOIN ModuleCalibrationModuleSet ON ModuleCalibrationModuleSet.ModuleCalibration = ModuleCalibration.id \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ? AND \
		( \
			SELECT count(*) \
			FROM ModuleCalibrationModuleSet AS inListGroup \
			JOIN otherModules ON \
				inListGroup.manufacturer = otherModules.manufacturer AND \
				inListGroup.typeNumber = otherModules.typeNumber AND \
				inListGroup.serialNumber = otherModules.serialNumber \
			WHERE ModuleCalibrationModuleSet.ModuleCalibration = inListGroup.ModuleCalibration \
		) = ? AND \
		( \
			SELECT count(*) \
			FROM ModuleCalibrationModuleSet AS listGroup \
			WHERE ModuleCalibrationModuleSet.ModuleCalibration = listGroup.ModuleCalibration AND ( \
				listGroup.manufacturer != ModuleCalibrationModuleSet.manufacturer OR \
				listGroup.typeNumber != ModuleCalibrationModuleSet.typeNumber OR \
				listGroup.serialNumber != ModuleCalibrationModuleSet.serialNumber \
			) \
		) = ?;";
		REXOS_INFO("%s", query.c_str());
		preparedStmt = connection->prepareStatement(query);
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		preparedStmt->setInt(4, moduleIdentifiers.size());
		preparedStmt->setInt(5, moduleIdentifiers.size());

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			REXOS_INFO("result...");
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
		delete result;
		delete preparedStmt;
		
		// delete the temp table for storing the modules
		preparedStmt = connection->prepareStatement("\
		DROP TEMPORARY TABLE otherModules;");
		preparedStmt->executeQuery();
		delete preparedStmt;
		
		return calibrationId;
	}
}
