#include <rexos_knowledge_database/Module.h>
#include <rexos_knowledge_database/KnowledgeDatabaseException.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>
#include <rexos_datatypes/ModuleIdentifier.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

#include "ros/ros.h"

namespace rexos_knowledge_database{
	Module::Module(rexos_datatypes::ModuleIdentifier moduleIdentifier) :
			ModuleType(moduleIdentifier), 
			moduleIdentifier(moduleIdentifier)
	{
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT * \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw KnowledgeDatabaseException("This module (" + moduleIdentifier.toString() + ") does not exist");
		}
		
		mountPointX = result->getInt("mountPointX");
		mountPointY = result->getInt("mountPointY");
		
		REXOS_DEBUG_STREAM("Constructed module with manufacturer=" << moduleIdentifier);
	}
	rexos_datatypes::ModuleIdentifier Module::getModuleIdentifier() {
		return moduleIdentifier;
	}
	std::string Module::getModuleProperties(){
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT moduleProperties \
		FROM Module \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current module (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string jsonProperties = result->getString("moduleProperties");
		
		return jsonProperties;
	}
	void Module::setModuleProperties(std::string jsonProperties) {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		UPDATE Module \
		SET moduleProperties = ? \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;"));
		preparedStmt->setString(1, jsonProperties);
		preparedStmt->setString(2, moduleIdentifier.getManufacturer());
		preparedStmt->setString(3, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(4, moduleIdentifier.getSerialNumber());
		preparedStmt->executeQuery();
	}
	Module* Module::getParentModule() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
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
		ASC LIMIT 1;"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		preparedStmt->setString(4, moduleIdentifier.getManufacturer());
		preparedStmt->setString(5, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(6, moduleIdentifier.getSerialNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1) {
			return NULL;
		} else {
			// set the cursor at the first result
			result->next();
			rexos_datatypes::ModuleIdentifier identifier(
				result->getString("manufacturer"),
				result->getString("typeNumber"),
				result->getString("serialNumber")
			);
			return new Module(identifier);
		}
	}
	std::vector<rexos_datatypes::ModuleIdentifier> Module::getChildModulesIdentifiers() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
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
		);"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		preparedStmt->setString(4, moduleIdentifier.getManufacturer());
		preparedStmt->setString(5, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(6, moduleIdentifier.getSerialNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::vector<rexos_datatypes::ModuleIdentifier> childModules;
		while(result->next()){
			rexos_datatypes::ModuleIdentifier identifier(
				result->getString("manufacturer"),
				result->getString("typeNumber"),
				result->getString("serialNumber")
			);
			childModules.push_back(identifier);
		}
		return childModules;
	}
	
	int Module::getMountPointX() {
		return mountPointX;
	}
	void Module::setMountPointX(int mountPointX) {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		UPDATE Module \
		SET mountPointY = ? \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;"));
		preparedStmt->setInt(1, mountPointX);
		preparedStmt->setString(2, moduleIdentifier.getManufacturer());
		preparedStmt->setString(3, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(4, moduleIdentifier.getSerialNumber());
		preparedStmt->executeQuery();
		
		this->mountPointX = mountPointX;
	}
	int Module::getMountPointY() {
		return mountPointY;		
	}
	void Module::setMountPointY(int mountPointY) {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		UPDATE Module \
		SET mountPointY = ? \
		WHERE manufacturer = ? AND \
		typeNumber = ? AND \
		serialNumber = ?;"));
		preparedStmt->setInt(1, mountPointY);
		preparedStmt->setString(2, moduleIdentifier.getManufacturer());
		preparedStmt->setString(3, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(4, moduleIdentifier.getSerialNumber());
		preparedStmt->executeQuery();
		
		this->mountPointY = mountPointY;
	}

	std::string Module::getCalibrationDataForModuleOnly() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
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
		) = 1;"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw KnowledgeDatabaseException("Unable to find calibration entry for only this module (there might be a calibration \
					entry shared with another module)");
		}
		// set the cursor at the first result
		result->next();
		return result->getString("properties");
	}
	std::string Module::getCalibrationDataForModuleAndChilds() {
		std::vector<rexos_datatypes::ModuleIdentifier> childs = getChildModulesIdentifiers();
		return getCalibrationDataForModuleAndOtherModules(childs);
	}
	std::string Module::getCalibrationDataForModuleAndOtherModules(std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiers) {
		int calibrationId = getCalibrationGroupForModuleAndOtherModules(moduleIdentifiers);
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT properties \
		FROM ModuleCalibration \
		WHERE id = ?;"));
		preparedStmt->setInt(1, calibrationId);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw KnowledgeDatabaseException("Unable to find calibration entry");
		}
		// set the cursor at the first result
		result->next();
		return result->getString("properties");
	}
	void Module::setCalibrationDataForModuleOnly(std::string properties) {
		try{
			std::vector<rexos_datatypes::ModuleIdentifier> emptyList;
			int calibrationId = getCalibrationGroupForModuleAndOtherModules(emptyList);
			
			// update existing entry
			std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
			UPDATE ModuleCalibration \
			SET properties = ? \
			WHERE id = ?;"));
			preparedStmt->setString(1, properties);
			preparedStmt->setInt(2, calibrationId);
			preparedStmt->executeQuery();
		} catch (KnowledgeDatabaseException ex) {
			// create a new entry
			std::unique_ptr<sql::PreparedStatement> preparedStmtA(connection->prepareStatement("\
			INSERT INTO ModuleCalibration (properties) \
			VALUES (?);"));
			preparedStmtA->setString(1, properties);
			preparedStmtA->executeQuery();
			
			std::unique_ptr<sql::PreparedStatement> preparedStmtB(connection->prepareStatement("\
			INSERT INTO ModuleCalibrationModuleSet (ModuleCalibration, manufacturer, typeNumber, serialNumber) \
			VALUES (LAST_INSERT_ID(), ?, ?, ?);"));
			
			preparedStmtB->setString(1, moduleIdentifier.getManufacturer());
			preparedStmtB->setString(2, moduleIdentifier.getTypeNumber());
			preparedStmtB->setString(3, moduleIdentifier.getSerialNumber());
			preparedStmtB->executeQuery();
		}
	}
	void Module::setCalibrationDataForModuleAndChilds(std::string properties) {
		std::vector<rexos_datatypes::ModuleIdentifier> childs = getChildModulesIdentifiers();
		setCalibrationDataForModuleAndOtherModules(childs, properties);
	}
	void Module::setCalibrationDataForModuleAndOtherModules(
			std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiers, std::string properties) {
		try{
			int calibrationId = getCalibrationGroupForModuleAndOtherModules(moduleIdentifiers);
			
			// update existing entry
			std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
			UPDATE ModuleCalibration \
			SET properties = ? \
			WHERE id = ?;"));
			preparedStmt->setString(1, properties);
			preparedStmt->setInt(2, calibrationId);
			preparedStmt->executeQuery();
		} catch (KnowledgeDatabaseException ex) {
			// create a new entry
			std::unique_ptr<sql::PreparedStatement> preparedStmtA(connection->prepareStatement("\
			INSERT INTO ModuleCalibration (properties) \
			VALUES (?);"));
			preparedStmtA->setString(1, properties);
			preparedStmtA->executeQuery();
			
			std::unique_ptr<sql::PreparedStatement> preparedStmtB(connection->prepareStatement("\
			INSERT INTO ModuleCalibrationModuleSet (ModuleCalibration, manufacturer, typeNumber, serialNumber) \
			VALUES (LAST_INSERT_ID(), ?, ?, ?);"));
			
			preparedStmtB->setString(1, moduleIdentifier.getManufacturer());
			preparedStmtB->setString(2, moduleIdentifier.getTypeNumber());
			preparedStmtB->setString(3, moduleIdentifier.getSerialNumber());
			preparedStmtB->executeQuery();
			
			for(uint i = 0; i < moduleIdentifiers.size(); i++){
				preparedStmtB->setString(1, moduleIdentifiers.at(i).getManufacturer());
				preparedStmtB->setString(2, moduleIdentifiers.at(i).getTypeNumber());
				preparedStmtB->setString(3, moduleIdentifiers.at(i).getSerialNumber());
				preparedStmtB->executeQuery();
			}
		}
	}
	
	
	int Module::getCalibrationGroupForModuleAndOtherModules(std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiers) {
		// create a temp table for storing the modules
		std::unique_ptr<sql::PreparedStatement> preparedStmtA(connection->prepareStatement("\
		CREATE TEMPORARY TABLE otherModules( \
			manufacturer char(200) NOT NULL, \
			typeNumber char(200) NOT NULL, \
			serialNumber char(200) NOT NULL \
		);"));
		preparedStmtA->executeQuery();
		
		// store the modules
		std::unique_ptr<sql::PreparedStatement> preparedStmtB(connection->prepareStatement("\
		INSERT INTO otherModules( \
			manufacturer, typeNumber, serialNumber \
		) VALUES ( \
			?, ?, ? \
		);"));
		for(uint i = 0; i < moduleIdentifiers.size(); i++){
			preparedStmtB->setString(1, moduleIdentifiers.at(i).getManufacturer());
			preparedStmtB->setString(2, moduleIdentifiers.at(i).getTypeNumber());
			preparedStmtB->setString(3, moduleIdentifiers.at(i).getSerialNumber());
			preparedStmtB->executeQuery();
		}
		
		// preform the actual query
		std::unique_ptr<sql::PreparedStatement> preparedStmtC(connection->prepareStatement("\
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
		) = ?;"));
		preparedStmtC->setString(1, moduleIdentifier.getManufacturer());
		preparedStmtC->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmtC->setString(3, moduleIdentifier.getSerialNumber());
		preparedStmtC->setInt(4, moduleIdentifiers.size());
		preparedStmtC->setInt(5, moduleIdentifiers.size());
		std::unique_ptr<sql::ResultSet> result(preparedStmtC->executeQuery());
		
		if(result->rowsCount() != 1){
			// delete the temp table for storing the modules
			std::unique_ptr<sql::PreparedStatement> preparedStmtD(connection->prepareStatement("\
			DROP TEMPORARY TABLE otherModules;"));
			preparedStmtD->executeQuery();
			throw KnowledgeDatabaseException("Unable to find calibration entry for only this module and other modules");
		}
		// set the cursor at the first result
		result->next();
		int calibrationId = result->getInt("id");
		
		// delete the temp table for storing the modules
		std::unique_ptr<sql::PreparedStatement> preparedStmtE(connection->prepareStatement("\
		DROP TEMPORARY TABLE otherModules;"));
		preparedStmtE->executeQuery();
		return calibrationId;
	}
}
