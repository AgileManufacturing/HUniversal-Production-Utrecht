#include <rexos_knowledge_database/GazeboCollision.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	std::vector<GazeboCollision> GazeboCollision::getCollisionsForModel(const GazeboModel& model) {
		std::shared_ptr<sql::Connection> connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT linkName, collisionName \
		FROM GazeboCollision \
		WHERE gazeboModel = ?;"));
		preparedStmt->setInt(1, model.getId());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::vector<GazeboCollision> output;
		while(result->next()) {
			output.push_back(GazeboCollision(model, result->getString("linkName"), result->getString("collisionName")));
		}
		return output;
	}
	GazeboCollision::GazeboCollision(const GazeboModel& model, std::string linkName, std::string collisionName) : 
		gazeboModel(model), linkName(linkName), collisionName(collisionName) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT maxForce, maxTorque, mayHaveContactWithChildModules \
		FROM GazeboCollision \
		WHERE gazeboModel = ? AND \
			linkName = ? AND \
			collisionName = ?;"));
		preparedStmt->setInt(1, model.getId());
		preparedStmt->setString(2, linkName);
		preparedStmt->setString(3, collisionName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current gazeboCollision (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		maxForce = result->getDouble("maxForce");
		maxTorque = result->getDouble("maxTorque");
		mayHaveContactWithChildModules = result->getBoolean("mayHaveContactWithChildModules");
	}
	
	GazeboModel GazeboCollision::getGazeboModel() const {
		return gazeboModel;
	}
	std::string GazeboCollision::getLinkName() const {
		return linkName;
	}
	std::string GazeboCollision::getCollisionName() const {
		return collisionName;
	}
	double GazeboCollision::getMaxForce() const {
		return maxForce;
	}
	double GazeboCollision::getMaxTorque() const {
		return maxTorque;
	}
	bool GazeboCollision::getMayHaveContactWithChildModules() const {
		return mayHaveContactWithChildModules;
	}
}
