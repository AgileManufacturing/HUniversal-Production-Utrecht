#include <rexos_knowledge_database/GazeboLink.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	std::vector<GazeboLink> GazeboLink::getLinksForModel(const GazeboModel& model) {
		std::shared_ptr<sql::Connection> connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT linkName \
		FROM GazeboLink \
		WHERE gazeboModel = ?;"));
		preparedStmt->setInt(1, model.getId());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::vector<GazeboLink> output;
		while(result->next()) {
			output.push_back(GazeboLink(model, result->getString("linkName")));
		}
		return output;
	}
	GazeboLink::GazeboLink(const GazeboModel& model, std::string linkName) : 
		gazeboModel(model), linkName(linkName) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT maxAcceleration \
		FROM GazeboLink \
		WHERE gazeboModel = ? AND \
			linkName = ?;"));
		preparedStmt->setInt(1, model.getId());
		preparedStmt->setString(2, linkName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current gazeboJoint (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		maxAcceleration = result->getDouble("maxAcceleration");
	}
	
	GazeboModel GazeboLink::getGazeboModel() const {
		return gazeboModel;
	}
	std::string GazeboLink::getLinkName() const {
		return linkName;
	}
	double GazeboLink::getMaxAcceleration() const {
		return maxAcceleration;
	}
}
