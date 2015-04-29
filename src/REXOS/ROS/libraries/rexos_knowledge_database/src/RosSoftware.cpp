#include <rexos_knowledge_database/RosSoftware.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	RosSoftware::RosSoftware(rexos_datatypes::ModuleTypeIdentifier moduleIdentifier) {
		connection = rexos_knowledge_database::connect();
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT id \
		FROM RosSoftware \
		WHERE id = ( \
			SELECT rosSoftware \
			FROM ModuleType \
			WHERE manufacturer = ? AND \
				typeNumber = ? \
		);");
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			delete result;
			delete preparedStmt;
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		id = result->getInt("id");
		
		delete result;
		delete preparedStmt;
	}
	RosSoftware::RosSoftware(std::string equipletName) {
		connection = rexos_knowledge_database::connect();
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT id \
		FROM RosSoftware \
		WHERE id = ( \
			SELECT rosSoftware \
			FROM Equiplet \
			WHERE name = ? \
		);");
		preparedStmt->setString(1, equipletName);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		id = result->getInt("id");
		
		delete result;
		delete preparedStmt;
	}
	
	std::istream* RosSoftware::getRosFile() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT zipFile \
		FROM RosSoftware \
		WHERE id = ?;");
		preparedStmt->setInt(1, id);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::istream* rosFile = result->getBlob("zipFile");
		
		delete result;
		delete preparedStmt;
		return rosFile;
	}
	std::string RosSoftware::getCommand() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT command \
		FROM RosSoftware \
		WHERE id = ?;");
		preparedStmt->setInt(1, id);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string command = result->getString("command");
		
		delete result;
		delete preparedStmt;
		return command;
	}
	int RosSoftware::getId() {
		return id;
	}
}
