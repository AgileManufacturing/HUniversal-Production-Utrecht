/**
 * @file FieldUpdateSubscription.h
 * @brief Subscription to changes to a specified field.
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

#ifndef FIELDUPDATESUBSCRIPTION_H_
#define FIELDUPDATESUBSCRIPTION_H_

#include "BasicOperationSubscription.h"
#include <vector>

namespace Blackboard {

/**
 * Different types of update operations that are available.
 */
enum MongoUpdateLogOperation {
	/**
	 * Represents a $set operation, e.g.: update({a:"a"}, {$set: {a : "b"}})
	 */
	SET,
	/**
	 * Represents an $unset operation, e.g.: update({a: "a"}, {$unset: {a : ""}})
	 */
	UNSET,
	/**
	 * Represents a replacement of the entire document, i.e. no $set or $unset is used.
	 * e.g.: update({a: "a"}, {b : "c"})
	 */
	REPLACE
};

/**
 * Subscription to changes to a specified field.
 */
class FieldUpdateSubscription: public Blackboard::BasicOperationSubscription {
public:
	/**
	 * Constructs a subscriptions for the specified field and subscriber.
	 */
	FieldUpdateSubscription(std::string fieldName, BlackboardSubscriber & subscriber);

	/**
	 * Adds an update operation to watch for. This method may be used multiple times in
	 * order to watch multiple update operations.
	 *
	 * \note Once the subscription has been added to the BlackboardClient, this method
	 * should no longer be used. Changes to the watched operations will not be reflected
	 * until the subscriptions is removed and re-added to the client.
	 *
	 * @param operation The MongoUpdateLogOperation that should be watched.
	 */
	void addOperation(MongoUpdateLogOperation operation);

	/**
	 * Removes an update operation.
	 *
	 * \note Once the subscription has been added to the BlackboardClient, this method
	 * should no longer be used. Changes to the watched operations will not be reflected
	 * until the subscriptions is removed and re-added to the client.
	 *
	 * @param operation The MongoUpdateLogOperation that should be watched.
	 */
	void removeOperation(MongoUpdateLogOperation operation);

	/**
	 * @see BasicOperationSubscription::matchesWithEntry(mongo::Query *)
	 */
	bool getQuery(mongo::Query * query_out) const;

	/**
	 * @see BasicOperationSubscription::matchesWithEntry(const OplogEntry&)
	 */
	bool matchesWithEntry(const OplogEntry& entry) const;

private:
	/**
	 * @var std::string fieldName
	 * Name of the field that should be watched.
	 */
	std::string fieldName;

	/**
	 * @var std::vector<MongoUpdateLogOperation> subscribedOperations
	 * List of operations that will be watched.
	 */
	std::vector<MongoUpdateLogOperation> subscribedOperations;

	/**
	 * Returns the operation string for the specified update operation, e.g. $set or $unset.
	 *
	 * @return The operation string for the specified update operation.
	 */
	std::string stringForUpdateOperation(MongoUpdateLogOperation operation) const;
};

} /* namespace Blackboard */
#endif /* FIELDUPDATESUBSCRIPTION_H_ */
