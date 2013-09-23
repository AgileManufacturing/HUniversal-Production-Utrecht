/**
 * @file OplogEntry.cpp
 * @brief Representation of a document in the oplog collection.
 * @date Created: 31 mei 2013
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
#include "rexos_blackboard_cpp_client/OplogEntry.h"
#include "rexos_blackboard_cpp_client/MongoOperation.h"

using namespace Blackboard;

const std::string OplogEntry::NAMESPACE_FIELD = "ns";

const std::string OplogEntry::TIMESTAMP_FIELD = "ts";

const std::string OplogEntry::OPERATION_FIELD = "op";

const std::string OplogEntry::UID_FIELD = "h";

const std::string OplogEntry::UPDATE_DOC_FIELD = "o";

const std::string OplogEntry::UPDATE_CRITERIA_FIELD = "o2";

const std::map<std::string, MongoOperation> OplogEntry::operationMap = {
		{ "i", MongoOperation::INSERT},
		{ "n", MongoOperation::NOOP},
		{ "d", MongoOperation::REMOVE},
		{ "u", MongoOperation::UPDATE}
};

OplogEntry::OplogEntry(mongo::BSONObj oplogEntry) : oplogEntry(oplogEntry) {}

MongoOperation OplogEntry::getOperation() const{
	return operationMap.at(oplogEntry.getStringField(OPERATION_FIELD.c_str()));
}

std::string OplogEntry::getNamespace() const{
	return std::string(oplogEntry.getStringField(NAMESPACE_FIELD.c_str()));
}

mongo::BSONObj OplogEntry::getUpdateDocument() const{
	return oplogEntry.getObjectField(UPDATE_DOC_FIELD.c_str());
}

mongo::BSONObj OplogEntry::getUpdateCriteria() const{
	return oplogEntry.getObjectField(UPDATE_CRITERIA_FIELD.c_str());
}

std::string OplogEntry::getOperationString(MongoOperation operation) {
	std::map<std::string, MongoOperation>::const_iterator iter;
	std::string operationString;
	for (iter = operationMap.begin() ; iter != operationMap.end() ; iter++) {
		if (iter->second == operation) {
			operationString = iter->first;
		}
	}

	return operationString;
}

bool OplogEntry::getTargetObjectId(mongo::OID & oid) const{
	mongo::BSONObj targetObj;
	if (oplogEntry.hasField(mongo::StringData(UPDATE_CRITERIA_FIELD))) {
		targetObj = oplogEntry.getObjectField(UPDATE_CRITERIA_FIELD.c_str());
	} else {
		targetObj = oplogEntry.getObjectField(UPDATE_DOC_FIELD.c_str());
	}

	mongo::BSONElement e;
	if (targetObj.getObjectID(e)) {
		oid = e.OID();
		return true;
	}

	return false;
}

std::string OplogEntry::toString() const {
	return oplogEntry.toString(false, false);
}

