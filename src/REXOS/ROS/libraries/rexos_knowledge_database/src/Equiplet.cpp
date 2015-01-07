#include <rexos_knowledge_database/Equiplet.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database{
	Equiplet::Equiplet(std::string name) :
		name(name)
	{
		connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
	}
	
	int Equiplet::getMountPointsX() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mountPointsX \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointsX = result->getInt("mountPointsX");
		
		delete result;
		delete preparedStmt;
		return mountPointsX;
	}

	int Equiplet::getMountPointsY() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mountPointsY \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointsY = result->getInt("mountPointsY");
		
		delete result;
		delete preparedStmt;
		return mountPointsY;
	}
	
	double Equiplet::getMountPointDistanceX() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mountPointsDistanceX \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointDistanceX = result->getDouble("mountPointDistanceX");
		
		delete result;
		delete preparedStmt;
		return mountPointDistanceX;
	}

	double Equiplet::getMountPointDistanceY() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mountPoinstDistanceY \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mountPointDistanceY = result->getDouble("mountPointDistanceY");
		
		delete result;
		delete preparedStmt;
		return mountPointDistanceY;
	}

	std::vector<rexos_datatypes::ModuleIdentifier> Equiplet::getModuleIdentifiersOfAttachedModules() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT manufacturer, typeNumber, serialNumber \
		FROM Module \
		WHERE equiplet = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		
		std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiersOfAttachedModules;
		while(result->next()) {
			rexos_datatypes::ModuleIdentifier identifier(
					result->getString("manufacturer"), result->getString("typeNumber"), result->getString("serialNumber"));
			moduleIdentifiersOfAttachedModules.push_back(identifier);
		}
		
		delete result;
		delete preparedStmt;
		return moduleIdentifiersOfAttachedModules;
	}

	std::vector<rexos_datatypes::ModuleIdentifier> Equiplet::getModuleIdentifiersOfAttachedModulesWithRosSoftware() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT Module.manufacturer, Module.typeNumber, Module.serialNumber \
		FROM Module \
		JOIN ModuleType ON Module.manufacturer = ModuleType.manufacturer AND \
			Module.typeNumber = ModuleType.typeNumber \
		WHERE Module.equiplet = ? AND \
			ModuleType.rosSoftware IS NOT NULL;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		
		std::vector<rexos_datatypes::ModuleIdentifier> moduleIdentifiersOfAttachedModules;
		while(result->next()) {
			rexos_datatypes::ModuleIdentifier identifier(
					result->getString("manufacturer"), result->getString("typeNumber"), result->getString("serialNumber"));
			moduleIdentifiersOfAttachedModules.push_back(identifier);
		}
		
		delete result;
		delete preparedStmt;
		return moduleIdentifiersOfAttachedModules;
	}

	std::string Equiplet::checkIfModuleStillExistInDatabase(std::string manufacturer, std::string typeNumber, std::string serialNumber){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT count(*) \
		FROM Module WHERE manufacturer = ? AND \
			typeNumber = ? AND \
			serialNumber = ? AND \
			Module.equiplet = ?");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);
		preparedStmt->setString(3, serialNumber);
		preparedStmt->setString(4, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		result->next();

		
		return result->getString("count(*)");
	}
}
