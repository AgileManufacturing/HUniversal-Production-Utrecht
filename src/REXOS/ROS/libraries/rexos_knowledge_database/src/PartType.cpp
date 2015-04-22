#include <rexos_knowledge_database/PartType.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	std::string PartType::getTypeNumberForParName(std::string partName) {
		std::unique_ptr<sql::Connection> connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT partType \
		FROM Part \
		WHERE partName = ?;");
		preparedStmt->setString(1, partName);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string partType = result->getString("partType");
		
		delete result;
		delete preparedStmt;
		return partType;
	}
	PartType::PartType(std::string typeNumber) :
			typeNumber(typeNumber)
	{
		connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
	}
	std::string PartType::getTypeNumber() {
		return typeNumber;
	}
	std::string PartType::getPartTypeProperties() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT partTypeProperties \
		FROM PartType \
		WHERE typeNumber = ?;");
		preparedStmt->setString(1, typeNumber);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current partType (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string jsonProperties = result->getString("partTypeProperties");
		
		delete result;
		delete preparedStmt;
		return jsonProperties;
	}
}
