/**
 * @file FieldUpdateSubscription.cpp
 * @brief 
 * @date Created: 3 jun. 2013
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

#include "rexos_blackboard_cpp_client/FieldUpdateSubscription.h"
#include "rexos_blackboard_cpp_client/OplogEntry.h"

namespace Blackboard {

FieldUpdateSubscription::FieldUpdateSubscription(std::string fieldName, BlackboardSubscriber & subscriber) :
		BasicOperationSubscription(MongoOperation::UPDATE, subscriber),
		fieldName(fieldName)
{
}

void FieldUpdateSubscription::addOperation(MongoUpdateLogOperation operation) {
	if (std::find(subscribedOperations.begin(), subscribedOperations.end(), operation) == subscribedOperations.end()) {
		subscribedOperations.push_back(operation);
	}
}

void FieldUpdateSubscription::removeOperation(MongoUpdateLogOperation operation) {
	std::vector<MongoUpdateLogOperation>::iterator iter =
			std::find(subscribedOperations.begin(), subscribedOperations.end(), operation);

	if (iter != subscribedOperations.end()) {
		subscribedOperations.erase(iter);
	}
}

bool FieldUpdateSubscription::getQuery(mongo::Query * query_out) const {
	if (!subscribedOperations.size()) {
		return false;
	}

	mongo::BSONArrayBuilder andArray;
	mongo::BSONArrayBuilder orArray;

	mongo::Query query;
	BasicOperationSubscription::getQuery(&query);
	andArray.append(query.obj);

	for (std::vector<MongoUpdateLogOperation>::const_iterator iter = subscribedOperations.begin(); iter != subscribedOperations.end() ; iter++) {
		std::stringstream fieldString;
		fieldString << "\""<< OplogEntry::UPDATE_DOC_FIELD << '.';

		std::string oper = stringForUpdateOperation(*iter);
		if (!oper.empty()) {
			fieldString << oper << '.';
		}

		fieldString << fieldName << "\"";

		orArray.append(BSON(fieldString.str() << BSON("$exists" << true)));
	}

	andArray.append(BSON("$or" << orArray.arr()));
	*query_out = QUERY("$and" << andArray.arr());
	return true;
}

bool FieldUpdateSubscription::matchesWithEntry(const OplogEntry& entry) const {
	if (!BasicOperationSubscription::matchesWithEntry(entry)) {
		return false;
	}

	mongo::BSONObj updateDoc = entry.getUpdateDocument();
	for (std::vector<MongoUpdateLogOperation>::const_iterator iter = subscribedOperations.begin(); iter != subscribedOperations.end() ; iter++) {
		std::string operationString = stringForUpdateOperation(*iter);
		if (!operationString.empty()) {
			if (updateDoc.hasField(operationString)) {
				mongo::BSONObj operationObj = updateDoc.getObjectField(operationString.c_str());
				if (operationObj.hasField(fieldName)) {
					return true;
				}
			}
		} else if (updateDoc.hasField(fieldName)) {
			return true;
		}
	}

	return false;
}

std::string FieldUpdateSubscription::stringForUpdateOperation(MongoUpdateLogOperation operation) const {
	switch(operation) {
	case MongoUpdateLogOperation::SET:
		return "$set";
	case MongoUpdateLogOperation::UNSET:
		return "$unset";
	default:
		return "";
	}
}

} /* namespace Blackboard */
