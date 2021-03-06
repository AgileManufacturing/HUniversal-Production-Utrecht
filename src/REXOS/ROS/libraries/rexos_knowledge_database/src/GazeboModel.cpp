#include <rexos_knowledge_database/GazeboModel.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	GazeboModel::GazeboModel(rexos_datatypes::ModuleTypeIdentifier moduleIdentifier) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT id, sdfFilename, parentLink, childLink, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ \
		FROM GazeboModel \
		WHERE id = ( \
			SELECT gazeboModel \
			FROM ModuleType \
			WHERE manufacturer = ? AND \
				typeNumber = ? \
		);"));
		preparedStmt->setString(1, moduleIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleIdentifier.getTypeNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current gazeboModel (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		id = result->getInt("id");
		sdfFilename = result->getString("sdfFilename");
		parentLink = result->getString("parentLink");
		childLink = result->getString("childLink");
		childLinkOffsetX = result->getDouble("childLinkOffsetX");
		childLinkOffsetY = result->getDouble("childLinkOffsetY");
		childLinkOffsetZ = result->getDouble("childLinkOffsetZ");
	}
	GazeboModel::GazeboModel(std::string equipletName) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT id, sdfFilename, parentLink, childLink, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ \
		FROM GazeboModel \
		WHERE id = ( \
			SELECT gazeboModel \
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
		sdfFilename = result->getString("sdfFilename");
		parentLink = result->getString("parentLink");
		childLink = result->getString("childLink");
		childLinkOffsetX = result->getDouble("childLinkOffsetX");
		childLinkOffsetY = result->getDouble("childLinkOffsetY");
		childLinkOffsetZ = result->getDouble("childLinkOffsetZ");
	}
	GazeboModel::GazeboModel(PartType& partType) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT id, sdfFilename, parentLink, childLink, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ \
		FROM GazeboModel \
		WHERE id = ( \
			SELECT gazeboModel \
			FROM PartType \
			WHERE typeNumber = ? \
		);"));
		preparedStmt->setString(1, partType.getTypeNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current rosSoftware (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		id = result->getInt("id");
		sdfFilename = result->getString("sdfFilename");
		parentLink = result->getString("parentLink");
		childLink = result->getString("childLink");
		childLinkOffsetX = result->getDouble("childLinkOffsetX");
		childLinkOffsetY = result->getDouble("childLinkOffsetY");
		childLinkOffsetZ = result->getDouble("childLinkOffsetZ");
	}
	
	std::istream* GazeboModel::getModelFile() const {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT zipFile \
		FROM GazeboModel \
		WHERE id = ?;"));
		preparedStmt->setInt(1, id);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current gazeboModel (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getBlob("zipFile");;
	}
	std::string GazeboModel::getSdfFilename() const {
		return sdfFilename;
	}
	std::string GazeboModel::getParentLink() const {
		return parentLink;
	}
	std::string GazeboModel::getChildLink() const {
		return childLink;
	}
	double GazeboModel::getChildLinkOffsetX() const {
		return childLinkOffsetX;
	}
	double GazeboModel::getChildLinkOffsetY() const {
		return childLinkOffsetY;
	}
	double GazeboModel::getChildLinkOffsetZ() const {
		return childLinkOffsetZ;
	}
	int GazeboModel::getId() const {
		return id;
	}
}
