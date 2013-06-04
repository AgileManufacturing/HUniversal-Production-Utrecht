/**
 * @file OplogEntry.h
 * @brief Representation of a document in the oplog collection.
 * @date Created: 28 mei 2013
 *
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#ifndef OPLOGENTRY_H_
#define OPLOGENTRY_H_

#include <string>
#include <map>
#include "mongo/bson/bson.h"
#include "rexos_blackboard_cpp_client/MongoOperation.h"

namespace Blackboard {

/**
 * Representation of a document in the oplog collection.
 */
class OplogEntry {
public:
	/**
	 * @var std::string NAMESPACE_FIELD
	 * Name of the namespace field within the oplog entry.
	 **/
	static const std::string NAMESPACE_FIELD;

	/**
	 * @var std::string TIMESTAMP_FIELD
	 * Name of the timestamp field within the oplog entry.
	 **/
	static const std::string TIMESTAMP_FIELD;

	/**
	 * @var std::string OPERATION_FIELD
	 * Name of the operation field within the oplog entry.
	 **/
	static const std::string OPERATION_FIELD;

	/**
	 * @var std::string UID_FIELD
	 * Name of the uid field within the oplog entry.
	 **/
	static const std::string UID_FIELD;

	/**
	 * @var std::string UPDATE_DOC_FIELD
	 * Name of the update document field within the oplog entry.
	 **/
	static const std::string UPDATE_DOC_FIELD;

	/**
	 * @var std::string UPDATE_CRITERIA_FIELD
	 * Name of the update criteria field within the oplog entry.
	 **/
	static const std::string UPDATE_CRITERIA_FIELD;

	/**
	 * Construct an OplogEntry object based on the specified entry.
	 * @param oplogEntry The DBObject containing the oplog data.
	 **/
	OplogEntry(mongo::BSONObj oplogEntry);

	/**
	 * Returns the operation of this oplog entry.
	 * @return the operation of this oplog entry.
	 **/
	MongoOperation getOperation() const;

	/**
	 * Returns the namespace of this oplog entry.
	 * @return the namespace of this oplog entry.
	 **/
	std::string getNamespace() const;

	/**
	 * Returns the update query (i.e. the query that was used to update the target documents) of this oplog entry.
	 * @return the update query of this oplog entry.
	 **/
	mongo::BSONObj getUpdateDocument() const;

	/**
	 * Returns the update criteria (i.e. the query used to match target documents) of this oplog entry.
	 * @return the update criteria of this oplog entry.
	 **/
	mongo::BSONObj getUpdateCriteria() const;

	/**
	 * Attempts to retrieve the ObjectId for the target document of the operation.
	 * @param oid Reference to an OID object where the value should be stored.
	 * @return true if an objectId could be extracted, false otherwise. When false is returned, oid is left unchanged.
	 **/
	bool getTargetObjectId(mongo::OID & oid) const;

	static std::string getOperationString(MongoOperation operation);

private:
	/**
	 * @var mongo::BSONObj oplogEntry
	 * Internal BSONObj representation of the oplog entry.
	 **/
	const mongo::BSONObj oplogEntry;

	/**
	 * @var std::map<std::string, MongoOperation> operationMap
	 * Internal mapping of the string representation of a MongoOperation to the MongoOperation object.
	 **/
	static const std::map<std::string, MongoOperation> operationMap;
};

}


#endif /* OPLOGENTRY_H_ */
