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
	
	std::string ModuleType::getModuleTypeProperties() {
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
		
		delete result;
		delete preparedStmt;
		return jsonProperties;
	}
	std::vector<TransitionPhase> ModuleType::getTransitionPhases() {
		std::vector<TransitionPhase> output;
		std::map<int, std::vector<RequiredMutation>> requiredMutations = getRequiredMutations();
		std::map<int, std::vector<SupportedMutation>> supportedMutations = getSupportedMutations();
		int currentPhase = 1;
		while(requiredMutations[currentPhase].size() != 0 || supportedMutations[currentPhase].size() != 0) {
			TransitionPhase transitionPhase(moduleTypeIdentifier, currentPhase, 
				requiredMutations[currentPhase], supportedMutations[currentPhase]);
			output.push_back(transitionPhase);
			currentPhase++;
		}
		return output;
	}
	
	std::map<int, std::vector<RequiredMutation>> ModuleType::getRequiredMutations() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT phase, mutation, isOptional \
		FROM RequiredCalibrationMutation \
		WHERE manufacturer = ? AND \
		typeNumber = ?;");
		preparedStmt->setString(1, moduleTypeIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleTypeIdentifier.getTypeNumber());

		sql::ResultSet* result = preparedStmt->executeQuery();
		std::map<int, std::vector<RequiredMutation>> requiredMutations;
		while(result->next() == true) {
			int phase = result->getInt("phase");
			RequiredMutation m = RequiredMutation(result->getString("mutation"), result->getBoolean("isOptional"));
			
			// insertion will be automatically performed when key does not exists, see http://en.cppreference.com/w/cpp/container/map/operator_at
			requiredMutations[phase].push_back(m);
			for(int i= 0 ; i < requiredMutations[phase].size(); i++) {
				std::cout << requiredMutations[phase].at(i);
			}
		}
		delete result;
		delete preparedStmt;
		return requiredMutations;
	}
	std::map<int, std::vector<SupportedMutation>> ModuleType::getSupportedMutations() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT phase, mutation \
		FROM SupportedCalibrationMutation \
		WHERE manufacturer = ? AND \
		typeNumber = ?;");
		preparedStmt->setString(1, moduleTypeIdentifier.getManufacturer());
		preparedStmt->setString(2, moduleTypeIdentifier.getTypeNumber());

		sql::ResultSet* result = preparedStmt->executeQuery();
		std::map<int, std::vector<SupportedMutation>> supportedMutations;
		while(result->next() == true) {
			int phase = result->getInt("phase");
			SupportedMutation m = SupportedMutation(result->getString("mutation"));
			
			// insertion will be automatically performed when key does not exists, see http://en.cppreference.com/w/cpp/container/map/operator_at
			supportedMutations[phase].push_back(m);
		}
		delete result;
		delete preparedStmt;
		return supportedMutations;
	}
}