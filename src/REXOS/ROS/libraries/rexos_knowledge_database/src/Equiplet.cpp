#include <rexos_knowledge_database/Equiplet.h>

#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

namespace rexos_knowledge_database{
	Equiplet::Equiplet(std::string name) :
				name(name)
	{
		connection = rexos_knowledge_database::connect();
	}
	
	int Equiplet::getMointPointsX(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPointsX \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointsX = result->getInt("mointPointsX");
		
		delete preparedStmt;
		return mointPointsX;
	}
	int Equiplet::getMointPointsY(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPointsY \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointsY = result->getInt("mointPointsY");
		
		delete preparedStmt;
		return mointPointsY;
	}
	
	double Equiplet::getMointPointDistanceX(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPointDistanceX \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointDistanceX = result->getDouble("mointPointDistanceX");
		
		delete preparedStmt;
		return mointPointDistanceX;
	}
	double Equiplet::getMointPointDistanceY(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT mointPointDistanceX \
		FROM Equiplet \
		WHERE name = ?;");
		preparedStmt->setString(1, name);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current equiplet (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		int mointPointDistanceY = result->getDouble("mointPointDistanceY");
		
		delete preparedStmt;
		return mointPointDistanceY;
	}
}