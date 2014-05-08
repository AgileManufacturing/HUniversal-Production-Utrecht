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
	
	int Equiplet::getMointPointsX() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPointsX \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointsX = result->getInt("mointPointsX");
		
		delete result;
		delete preparedStmt;
		return mointPointsX;
	}
	int Equiplet::getMointPointsY() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPointsY \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointsY = result->getInt("mointPointsY");
		
		delete result;
		delete preparedStmt;
		return mointPointsY;
	}
	
	double Equiplet::getMointPointDistanceX() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPointsDistanceX \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointDistanceX = result->getDouble("mointPointDistanceX");
		
		delete result;
		delete preparedStmt;
		return mointPointDistanceX;
	}
	double Equiplet::getMointPointDistanceY() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPoinstDistanceY \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointDistanceY = result->getDouble("mointPointDistanceY");
		
		delete result;
		delete preparedStmt;
		return mointPointDistanceY;
	}
	std::vector<ModuleIdentifier> Equiplet::getModuleIdentifiersOfAttachedModules() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT manufacturer, typeNumber, serialNumber \
		FROM Module \
		WHERE equiplet = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		
		std::vector<ModuleIdentifier> moduleIdentifiersOfAttachedModules;
		while(result->next()) {
			ModuleIdentifier identifier = ModuleIdentifier(
					result->getString("manufacturer"), result->getString("typeNumber"), result->getString("serialNumber"));
			moduleIdentifiersOfAttachedModules.push_back(identifier);
		}
		
		delete result;
		delete preparedStmt;
		return moduleIdentifiersOfAttachedModules;
	}
}