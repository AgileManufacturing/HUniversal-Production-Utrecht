#include <rexos_knowledge_database/ModuleType.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database{
	ModuleType::ModuleType(ModuleTypeIdentifier moduleTypeIdentifier) :
				moduleTypeIdentifier(moduleTypeIdentifier)
	{
		connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
	}
	
	std::string ModuleType::getModuleTypeProperties(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT moduleTypeProperties \
		FROM ModuleType \
		WHERE manufacturer = ? AND \
		typeNumber = ?;");
		preparedStmt->setString(1, moduleTypeIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleTypeIdentifier.getTypeNumber());

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