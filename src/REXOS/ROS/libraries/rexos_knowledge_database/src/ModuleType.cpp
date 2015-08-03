#include <rexos_knowledge_database/ModuleType.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database{
	ModuleType::ModuleType(rexos_datatypes::ModuleTypeIdentifier moduleTypeIdentifier) :
			moduleTypeIdentifier(moduleTypeIdentifier)
	{
		connection = rexos_knowledge_database::connect();
	}
	
	std::string ModuleType::getModuleTypeProperties() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT moduleTypeProperties \
		FROM ModuleType \
		WHERE manufacturer = ? AND \
		typeNumber = ?;"));
		preparedStmt->setString(1, moduleTypeIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleTypeIdentifier.getTypeNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current moduleType (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getString("moduleTypeProperties");
	}
	std::vector<rexos_datatypes::TransitionPhase> ModuleType::getTransitionPhases() {
		std::vector<rexos_datatypes::TransitionPhase> output;
		std::map<int, std::vector<rexos_datatypes::RequiredMutation>> requiredMutations = getRequiredMutations();
		std::map<int, std::vector<rexos_datatypes::SupportedMutation>> supportedMutations = getSupportedMutations();
		int currentPhase = 1;
		while(requiredMutations[currentPhase].size() != 0 || supportedMutations[currentPhase].size() != 0) {
			rexos_datatypes::TransitionPhase transitionPhase(moduleTypeIdentifier, currentPhase, 
				requiredMutations[currentPhase], supportedMutations[currentPhase]);
			output.push_back(transitionPhase);
			currentPhase++;
		}
		return output;
	}
	
	std::map<int, std::vector<rexos_datatypes::RequiredMutation>> ModuleType::getRequiredMutations() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT phase, mutation, isOptional \
		FROM RequiredCalibrationMutation \
		WHERE manufacturer = ? AND \
		typeNumber = ?;"));
		preparedStmt->setString(1, moduleTypeIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleTypeIdentifier.getTypeNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::map<int, std::vector<rexos_datatypes::RequiredMutation>> requiredMutations;
		while(result->next() == true) {
			int phase = result->getInt("phase");
			rexos_datatypes::RequiredMutation m(result->getString("mutation"), result->getBoolean("isOptional"));
			
			// insertion will be automatically performed when key does not exists, see http://en.cppreference.com/w/cpp/container/map/operator_at
			requiredMutations[phase].push_back(m);
		}
		return requiredMutations;
	}
	std::map<int, std::vector<rexos_datatypes::SupportedMutation>> ModuleType::getSupportedMutations() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT phase, mutation \
		FROM SupportedCalibrationMutation \
		WHERE manufacturer = ? AND \
		typeNumber = ?;"));
		preparedStmt->setString(1, moduleTypeIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleTypeIdentifier.getTypeNumber());
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::map<int, std::vector<rexos_datatypes::SupportedMutation>> supportedMutations;
		while(result->next() == true) {
			int phase = result->getInt("phase");
			rexos_datatypes::SupportedMutation m(result->getString("mutation"));
			
			// insertion will be automatically performed when key does not exists, see http://en.cppreference.com/w/cpp/container/map/operator_at
			supportedMutations[phase].push_back(m);
		}
		return supportedMutations;
	}
}
