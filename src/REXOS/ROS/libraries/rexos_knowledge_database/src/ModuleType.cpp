#include <rexos_knowledge_database/ModuleType.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database{
	ModuleType::ModuleType(std::string manufacturer, std::string typeNumber) :
				manufacturer(manufacturer), typeNumber(typeNumber)
	{
		connection = rexos_knowledge_database::connect();
	}
	
	std::string ModuleType::getModuleTypeProperties(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT moduleTypeProperties \
		FROM ModuleType \
		WHERE manufacturer = ? AND \
		typeNumber = ?;");
		preparedStmt->setString(1, manufacturer);
		preparedStmt->setString(2, typeNumber);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current moduleType (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string jsonProperties = result->getString("moduleTypeProperties");
		
		delete preparedStmt;
		return jsonProperties;
	}
}