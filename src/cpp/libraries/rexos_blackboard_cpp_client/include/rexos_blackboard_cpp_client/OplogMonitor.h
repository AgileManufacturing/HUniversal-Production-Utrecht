/**
 * @file OplogMonitor.h
 * @brief Monitors the oplog and dispatches callbacks for active subscriptions.
 * @date Created: 4 jun. 2013
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

#ifndef OPLOGMONITOR_H_
#define OPLOGMONITOR_H_

#include <string>
#include <vector>
#include "mongo/client/dbclientinterface.h"
#include "boost/thread.hpp"

namespace Blackboard {

class BlackboardSubscription;

/**
 * Monitors the oplog and dispatches callbacks for active subscriptions.
 */
class OplogMonitor {
public:
	/**
	 * Constructs an OplogMonitor for the specified host.
	 *
	 * @param host String containg the hostname of the server, e.g. localhost:27017
	 * @param oplogDBName The database name where the oplog collection resides, typically local.
	 * @param oplogCollectionName The name of the oplog collection, typically oplog.rs.
	 */
	OplogMonitor(
			std::string host,
			std::string oplogDBName,
			std::string oplogCollectionName);

	/**
	 * Destructs the OplogMonitor and stops the current thread (if any).
	 */
	~OplogMonitor();

	/**
	 * Adds a subscription to be watched.
	 *
	 * @param sub Reference to the subscription that should be watched.
	 */
	void addSubscription(BlackboardSubscription& sub);

	/**
	 * Removes a watched subscription.
	 *
	 * @param sub Reference to the subscription that should be removed.
	 */
	void removeSubscription(BlackboardSubscription& sub);

	/**
	 * Sets the namespace of the database and collection that should be watched.
	 *
	 * @param database The name of the database.
	 * @param collection The name of the collection.
	 */
	void setNamespace(std::string &database, std::string &collection);

	/**
	 * Sets the namespace of the database and collection that should be watched.
	 *
	 * @param omtNamespace The namespace of the collection, i.e. "database.collection".
	 */
	void setNamespace(std::string omtNamespace);

private:
	/**
	 * Starts a new thread that will check for updates for all of the registered subscriptions.
	 */
	void start();

	/**
	 * Interrupts the current thread and kills its cursor.
	 */
	void interrupt();

	/**
	 * Interrupts the current thread and starts a new one.
	 */
	void restart();

	/**
	 * This method is used as the thread's main method to monitor the oplog and handle subscriptions.
	 */
	void run();

	/**
	 * Creates the query that should be used to filter interesting events from the oplog based on the current subscriptions.
	 *
	 * @return Query representing all current subscriptions.
	 */
	mongo::Query createOplogQuery();

	/**
	 * @var long long int currentCursorId
	 * ID of the cursor that is currently being used.
	 */
	long long int currentCursorId;

	/**
	 * @var std::vector<BlackboardSubscription *> subscriptions
	 * Vector holding all the subscriptions that have been registered.
	 */
	std::vector<BlackboardSubscription *> subscriptions;

	/**
	 * @var std::string host
	 * Hostname of the server.
	 */
	std::string host;

	/**
	 * @var std::string oplogNamespace
	 * Namespace of the oplog collection.
	 */
	std::string oplogNamespace;

	/**
	 * @var std::string omtNamespace
	 * The namespace of the database and collection that should be watched.
	 */
	std::string omtNamespace;

	/**
	 * @var boost::mutex subscriptionsMutex
	 * Mutex used to protect the subscriptions vector.
	 */
	boost::mutex subscriptionsMutex;

	/**
	 * @var boost::thread *currentThread;
	 * Pointer to the current thread.
	 */
	boost::thread *currentThread;
};

} /* namespace Blackboard */
#endif /* OPLOGMONITOR_H_ */
