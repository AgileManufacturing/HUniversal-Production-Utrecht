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
		connection = rexos_knowledge_database::connect();
		
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT * \
		FROM Part \
		WHERE partName = ?;"));
		preparedStmt->setString(1, partName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw KnowledgeDatabaseException("This part (" + partName + ") does not exist");
		}
		// set the cursor at the first result
		result->next();
		positionX = result->getDouble("positionX");
		positionY = result->getDouble("positionY");
		positionZ = result->getDouble("positionZ");
		rotationX = result->getDouble("rotationX");
		rotationY = result->getDouble("rotationY");
		rotationZ = result->getDouble("rotationZ");
	}
	std::string Part::getPartName() {
		return partName;
	}
	std::string Part::getPartProperties(){
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT partProperties \
		FROM Part \
		WHERE partName = ?;"));
		preparedStmt->setString(1, partName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->getString("partProperties");
	}
	void Part::setPartProperties(std::string jsonProperties) {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		UPDATE Part \
		SET partProperties = ? \
		WHERE partName = ?;"));
		preparedStmt->setString(1, partName);
		preparedStmt->executeQuery();
	}
	Part* Part::getParentPart() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
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
		ASC LIMIT 1;"));
		preparedStmt->setString(1, partName);
		preparedStmt->setString(2, partName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1) {
			return NULL;
		} else {
			// set the cursor at the first result
			result->next();
			std::string partNameOfParent(result->getString("partName"));
			return new Part(partNameOfParent);
		}
	}
	std::vector<std::string> Part::getChildPartNames() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
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
			ancestor.partName IS NULL;"));
		preparedStmt->setString(1, partName);
		preparedStmt->setString(2, partName);
		preparedStmt->setString(3, partName);
		preparedStmt->setString(4, partName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		std::vector<std::string> childPartNames;
		while(result->next()){
			childPartNames.push_back(result->getString("partName"));
		}
		return childPartNames;
	}
	bool Part::hasQrCodeFile() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT qrCode \
		FROM Part \
		WHERE partName = ?;"));
		preparedStmt->setString(1, partName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		return result->isNull("qrCode") ? false : true;
	}
	std::istream* Part::getQrCodeFile() {
		std::unique_ptr<sql::PreparedStatement> preparedStmt(connection->prepareStatement("\
		SELECT qrCode \
		FROM Part \
		WHERE partName = ?;"));
		preparedStmt->setString(1, partName);
		std::unique_ptr<sql::ResultSet> result(preparedStmt->executeQuery());
		
		if(result->rowsCount() != 1){
			throw std::runtime_error("Unable to find current part (someone deleted this instance in the database)");
		}
		// set the cursor at the first result
		result->next();
		std::istream* qrCodeFile = result->getBlob("qrCode");
		
		return qrCodeFile;
	}
}
