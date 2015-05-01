#include <rexos_knowledge_database/PartType.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	std::string PartType::getTypeNumberForParName(std::string partName) {
		std::shared_ptr<sql::Connection> connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT partType \
		FROM Part \
		WHERE partName = ?;"));
		preparedStmt->setString(1, partName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getString("partType");
	}
	
	PartType::PartType(std::string typeNumber) :
			typeNumber(typeNumber)
	{
		connection = rexos_knowledge_database::connect();
	}
	std::string PartType::getTypeNumber() {
		return typeNumber;
	}
	std::string PartType::getPartTypeProperties() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT partTypeProperties \
		FROM PartType \
		WHERE typeNumber = ?;"));
		preparedStmt->setString(1, typeNumber);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current partType (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getString("partTypeProperties");
	}
}
