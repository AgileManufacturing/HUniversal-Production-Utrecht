#include <rexos_knowledge_database/GazeboJoint.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database {
	std::vector<GazeboJoint> GazeboJoint::getJointsForModel(const GazeboModel& model) {
		std::shared_ptr<sql::Connection> connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT jointName \
		FROM GazeboJoint \
		WHERE gazeboModel = ?;"));
		preparedStmt->setInt(1, model.getId());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::vector<GazeboJoint> output;
		while(result->next()) {
			output.push_back(GazeboJoint(model, result->getString("jointName")));
		}
		return output;
	}
	GazeboJoint::GazeboJoint(const GazeboModel& model, std::string jointName) : 
		gazeboModel(model), jointName(jointName) {
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT maxErrorPose \
		FROM GazeboJoint \
		WHERE gazeboModel = ? AND \
			jointName = ?;"));
		preparedStmt->setInt(1, model.getId());
		preparedStmt->setString(2, jointName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current gazeboJoint (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		maxErrorPose = result->getDouble("maxErrorPose");
	}
	
	GazeboModel GazeboJoint::getGazeboModel() const {
		return gazeboModel;
	}
	std::string GazeboJoint::getJointName() const {
		return jointName;
	}
	double GazeboJoint::getMaxErrorPose() const {
		return maxErrorPose;
	}
}
