/**
 * @file BlackboardSubscription.h
 * @brief Abstract base class defining the basic structure a blackboard subscription should adhere to.
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

#ifndef BLACKBOARDSUBSCRIPTION_H_
#define BLACKBOARDSUBSCRIPTION_H_

#include <string>
#include "mongo/client/dbclientinterface.h"

namespace Blackboard {

class OplogEntry;
class BlackboardSubscriber;

/**
 * Abstract base class defining the basic structure a blackboard subscription should adhere to.
 */
class BlackboardSubscription {
public:
	/**
	 * Creates the query that should be used to find documents matching the subscription.
	 *
	 * @param query_out Pointer to a buffer that will hold the constructed mongo::Query object.
	 */
	virtual bool getQuery(mongo::Query * query_out) const = 0;

	/**
	 * Checks if the subscriptions matches the given oplogEntry.
	 *
	 * @param oplogEntry Reference to the oplogEntry describing the event.
	 *
	 * @return true if the subscription matches the oplogEntry, false otherwise.
	 */
	virtual bool matchesWithEntry(const OplogEntry & oplogEntry) const = 0;

	/**
	 * Returns the subscriber for this subscription.
	 *
	 * @return Reference to the subscriber associated with this subscription.
	 */
	BlackboardSubscriber & getSubscriber() const;

	/**
	 * Constructs a new subscription object with the specified subscriber.
	 *
	 * @param subscriber Reference to the object that should receive a callback for this subscription.
	 */
	BlackboardSubscription(BlackboardSubscriber & subscriber);

	/**
	 * Virtual destructor to allow child classes to clean up properly.
	 */
	virtual ~BlackboardSubscription() {};

private:
	/**
	 * Reference to the subscriber associated with this subscription.
	 */
	BlackboardSubscriber & subscriber;
};

} /* namespace Blackboard */
#endif /* BLACKBOARDSUBSCRIPTION_H_ */
