#include <rexos_knowledge_database/Part.h>
#include <rexos_knowledge_database/KnowledgeDatabaseException.h>
#include <rexos_knowledge_database/rexos_knowledge_database.h>

#include <cppconn/resultset.h>
#include <cppconn/prepared_statement.h>

#include "ros/ros.h"

namespace rexos_knowledge_database{
	Part::Part(std::string partName) :
			PartType(PartType::getTypeNumberForParName(partName)), partName(partName)
	{
		connection = std::unique_ptr<sql::Connection>(rexos_knowledge_database::connect());
		
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT * \
		FROM Part \
		WHERE partName = ?;");
		preparedStmt->setString(1, partName);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			std::string message = "This part (" + partName + ") does not exist";
			throw KnowledgeDatabaseException(message.c_str());
		}
		// set the cursor at the first result
		result->next();
		positionX = result->getDouble("positionX");
		positionY = result->getDouble("positionY");
		positionZ = result->getDouble("positionZ");
		rotationX = result->getDouble("rotationX");
		rotationY = result->getDouble("rotationY");
		rotationZ = result->getDouble("rotationZ");
		
		delete result;
		delete preparedStmt;
	}
	std::string Part::getPartName() {
		return partName;
	}
	std::string Part::getPartProperties(){
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT partProperties \
		FROM Part \
		WHERE partName = ?;");
		preparedStmt->setString(1, partName);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::string jsonProperties = result->getString("partProperties");
		
		delete result;
		delete preparedStmt;
		return jsonProperties;
	}
	void Part::setPartProperties(std::string jsonProperties) {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		UPDATE Part \
		SET partProperties = ? \
		WHERE partName = ?;");
		preparedStmt->setString(1, partName);

		preparedStmt->executeQuery();
		delete preparedStmt;
	}
	Part* Part::getParentPart() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT partName \
		FROM Part \
		WHERE attachedToLeft < (\
			SELECT attachedToLeft FROM Part \
			WHERE partName = ? \
		) AND attachedToRight > (\
	 		SELECT attachedToRight FROM Part \
			WHERE partName = ? \
		)\
		ORDER BY abs(attachedToLeft - attachedToRight) \
		ASC LIMIT 1;");
		preparedStmt->setString(1, partName);
		preparedStmt->setString(2, partName);

		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1) {
			delete result;
			delete preparedStmt;
			return NULL;
		} else {
			// set the cursor at the first result
			result->next();
			std::string partNameOfParent(result->getString("partName"));
			delete result;
			delete preparedStmt;
			return new Part(partNameOfParent);
		}
	}
	std::vector<std::string> Part::getChildPartNames() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT child.partName \
		FROM Part AS child \
		LEFT JOIN Part AS ancestor ON \
			ancestor.attachedToLeft BETWEEN ( \
				SELECT attachedToLeft FROM Part WHERE partName = ? \
			) + 1 AND ( \
				SELECT attachedToRight FROM Part WHERE partName = ? \
			) - 1 AND \
			child.attachedToLeft BETWEEN ancestor.attachedToLeft+ 1 AND ancestor.attachedToRight - 1 \
		WHERE \
			child.attachedToLeft BETWEEN ( \
				SELECT attachedToLeft FROM Part WHERE partName = ? \
			) + 1 AND ( \
				SELECT attachedToRight FROM Part WHERE partName = ? \
			) - 1 AND \
			ancestor.partName IS NULL;");
		preparedStmt->setString(1, partName);
		preparedStmt->setString(2, partName);
		preparedStmt->setString(3, partName);
		preparedStmt->setString(4, partName);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		std::vector<std::string> childPartNames;
		if(result->rowsCount() != 0){
			// get all the childs
			while(result->next()){
				childPartNames.push_back(result->getString("partName"));
			}
		}
		delete result;
		delete preparedStmt;
		return childPartNames;
	}
	bool Part::hasQrCodeFile() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT qrCode \
		FROM Part \
		WHERE partName = ?;");
		preparedStmt->setString(1, partName);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		bool isNull = result->isNull("qrCode");
		
		delete result;
		delete preparedStmt;
		if(isNull == true) return false;
		else return true;
	}
	std::istream* Part::getQrCodeFile() {
		sql::PreparedStatement* preparedStmt = connection->prepareStatement("\
		SELECT qrCode \
		FROM Part \
		WHERE partName = ?;");
		preparedStmt->setString(1, partName);
		
		sql::ResultSet* result = preparedStmt->executeQuery();
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::istream* qrCodeFile = result->getBlob("qrCode");
		
		delete result;
		delete preparedStmt;
		return qrCodeFile;
	}
}