#include <rexos_knowledge_database/Equiplet.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database{
	Equiplet::Equiplet(std::string name) :
		name(name)
	{
		connection = rexos_knowledge_database::connect();
	}
	
	int Equiplet::getMountPointsX() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT mountPointsX \
		FROM Equiplet \
		WHERE name = ?;"));
		preparedStmt->setString(1, name);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getInt("mountPointsX");
	}

	int Equiplet::getMountPointsY() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT mountPointsY \
		FROM Equiplet \
		WHERE name = ?;"));
		preparedStmt->setString(1, name);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getInt("mountPointsY");
	}
	
	double Equiplet::getMountPointDistanceX() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT mountPointDistanceX \
		FROM Equiplet \
		WHERE name = ?;"));
		preparedStmt->setString(1, name);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getDouble("mountPointDistanceX");
	}

	double Equiplet::getMountPointDistanceY() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT mountPointDistanceY \
		FROM Equiplet \
		WHERE name = ?;"));
		preparedStmt->setString(1, name);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getDouble("mountPointDistanceY");
	}

	std::vector<rexos_datatypes::ModuleIdentifier> Equiplet::getModuleIdentifiersOfAttachedModules() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT manufacturer, typeNumber, serialNumber \
		FROM Module \
		WHERE equiplet = ?;"));
		preparedStmt->setString(1, name);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiersOfAttachedModules;
		while(result->next()) {
			rexos_datatypes::ModuleIdentifier identifier(
					result->getString("manufacturer"), result->getString("typeNumber"), result->getString("serialNumber"));
			moduleIdentifiersOfAttachedModules.push_back(identifier);
		}
		
		return moduleIdentifiersOfAttachedModules;
	}

	std::vector<rexos_datatypes::ModuleIdentifier> Equiplet::getModuleIdentifiersOfAttachedModulesWithRosSoftware() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT Module.manufacturer, Module.typeNumber, Module.serialNumber \
		FROM Module \
		JOIN ModuleType ON Module.manufacturer = ModuleType.manufacturer AND \
			Module.typeNumber = ModuleType.typeNumber \
		WHERE Module.equiplet = ? AND \
			ModuleType.rosSoftware IS NOT NULL;"));
		preparedStmt->setString(1, name);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiersOfAttachedModules;
		while(result->next()) {
			rexos_datatypes::ModuleIdentifier identifier(
					result->getString("manufacturer"), result->getString("typeNumber"), result->getString("serialNumber"));
			moduleIdentifiersOfAttachedModules.push_back(identifier);
		}
		
		return moduleIdentifiersOfAttachedModules;
	}

	bool Equiplet::checkIfModuleStillExistInDatabase(rexos_datatypes::ModuleIdentifier moduleIdentifier) {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT * \
		FROM Module \
		WHERE manufacturer = ? AND \
			typeNumber = ? AND \
			serialNumber = ? AND \
			equiplet = ?;"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		preparedStmt->setString(3, moduleIdentifier.getSerialNumber());
		preparedStmt->setString(4, name);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() == 1) return true;
		else return false;
	}
}
