#include <rexos_knowledge_database/RosSoftware.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	RosSoftware::RosSoftware(rexos_datatypes::ModuleTypeIdentifier moduleIdentifier) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT id \
		FROM RosSoftware \
		WHERE id = ( \
			SELECT rosSoftware \
			FROM ModuleType \
			WHERE manufacturer = ? AND \
				typeNumber = ? \
		);"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		id = result->getInt("id");
	}
	RosSoftware::RosSoftware(std::string equipletName) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT id \
		FROM RosSoftware \
		WHERE id = ( \
			SELECT rosSoftware \
			FROM Equiplet \
			WHERE name = ? \
		);"));
		preparedStmt->setString(1, equipletName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		id = result->getInt("id");
	}
	
	std::istream* RosSoftware::getRosFile() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT zipFile \
		FROM RosSoftware \
		WHERE id = ?;"));
		preparedStmt->setInt(1, id);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getBlob("zipFile");
	}
	std::string RosSoftware::getCommand() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT command \
		FROM RosSoftware \
		WHERE id = ?;"));
		preparedStmt->setInt(1, id);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getString("command");
	}
	int RosSoftware::getId() {
		return id;
	}
}
