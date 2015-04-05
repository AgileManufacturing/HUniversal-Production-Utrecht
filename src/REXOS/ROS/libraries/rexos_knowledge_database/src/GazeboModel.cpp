#include <rexos_knowledge_database/GazeboModel.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	GazeboModel::GazeboModel(rexos_datatypes::ModuleTypeIdentifier moduleIdentifier) {
		connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT id, sdfFilename, parentLink, childLink, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ \
		FROM GazeboModel \
		WHERE id = ( \
			SELECT gazeboModel \
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
		
		delete result;
		delete preparedStmt;
	}
	GazeboModel::GazeboModel(std::string equipletName) {
		connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT id, sdfFilename, parentLink, childLink, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ \
		FROM GazeboModel \
		WHERE id = ( \
			SELECT gazeboModel \
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
		sdfFilename = result->getString("sdfFilename");
		parentLink = result->getString("parentLink");
		childLink = result->getString("childLink");
		childLinkOffsetX = result->getDouble("childLinkOffsetX");
		childLinkOffsetY = result->getDouble("childLinkOffsetY");
		childLinkOffsetZ = result->getDouble("childLinkOffsetZ");
		
		delete result;
		delete preparedStmt;
	}
	
	std::istream* GazeboModel::getModelFile() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT zipFile \
		FROM GazeboModel \
		WHERE id = ?;");
		preparedStmt->setInt(1, id);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current gazeboModel (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::istream* rosFile = result->getBlob("zipFile");
		
		delete result;
		delete preparedStmt;
		return rosFile;
	}
	std::string GazeboModel::getSdfFilename() {
		return sdfFilename;
	}
	std::string GazeboModel::getParentLink() {
		return parentLink;
	}
	std::string GazeboModel::getChildLink() {
		return childLink;
	}
	double GazeboModel::getChildLinkOffsetX() {
		return childLinkOffsetX;
	}
	double GazeboModel::getChildLinkOffsetY() {
		return childLinkOffsetY;
	}
	double GazeboModel::getChildLinkOffsetZ() {
		return childLinkOffsetZ;
	}
	int GazeboModel::getId() {
		return id;
	}
}
