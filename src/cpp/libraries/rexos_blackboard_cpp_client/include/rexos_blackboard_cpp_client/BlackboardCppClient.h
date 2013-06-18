/**
 * @file BlackboardCppClient.h
 * @brief Represents a blackboard connection.
 * @date Created: 2012-10-12
 *
 * @author Dennis Koole
 * @author Jan-Willem Willebrands
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012-2013, HU University of Applied Sciences Utrecht.
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

#ifndef BLACKBOARD_CPP_CLIENT_H_
#define BLACKBOARD_CPP_CLIENT_H_

#include <string>
#include "mongo/client/dbclientinterface.h"

namespace Blackboard {
class OplogMonitor;
class BlackboardSubscription;

/**
 * Represents a blackboard connection.
 **/
class BlackboardCppClient{
public:
	/**
	 * Constructs a new client with the specified settings.
	 *
	 * @param hostname The server's hostname.
	 * @param db Name of the database that should be used.
	 * @param coll Name of the collection that should be used.
	 */
	BlackboardCppClient(const std::string &hostname, const std::string db, const std::string coll);

	/**
	 * Constructs a new client with the specified settings.
	 *
	 * @param hostname The server's hostname.
	 * @param port The port that should be used.
	 * @param db Name of the database that should be used.
	 * @param coll Name of the collection that should be used.
	 */
	BlackboardCppClient(const std::string &hostname, int port, const std::string db, const std::string coll);

	/**
	 * Destroys the client and its oplog monitor.
	 */
	virtual ~BlackboardCppClient();

	/**
	 * Sets the database to be used.
	 *
	 * @note Changing the namespace (i.e. database or collection) of the client causes
	 * any running monitor thread to stop. In addition, all subscriptions are removed from the client.
	 *
	 * @param db Name of the database.
	 */
	void setDatabase(const std::string &db);

	/**
	 * Sets the collection to be used.
	 * If a monitor thread is active, it is stopped and all subscriptions are removed.
	 *
	 * @note Changing the namespace (i.e. database or collection) of the client causes
	 * any running monitor thread to stop. In addition, all subscriptions are removed from the client.
	 *
	 * @param col Name of the collection that should be used.
	 */
	void setCollection(const std::string &col);

	/**
	 * Adds the specified subscription to be watched.
	 *
	 * @param sub The subscription that should be watched.
	 */
	void subscribe(BlackboardSubscription &sub);

	/**
	 * Removes the specified subscription.
	 *
	 * @param sub The subscription that should be removed.
	 */
	void unsubscribe(BlackboardSubscription &sub);

	/**
	 * Inserts a document into the database.
	 *
	 * @param json JSON string representation of the object that should be inserted.
	 */
	void insertDocument(std::string json);

	/**
	 * Removes documents matching the specified query from the database.
	 *
	 * @param queryAsJSON The query that is used to determine which objects will be removed.
	 */
	void removeDocuments(std::string queryAsJSON);

	/**
	 * Queries the blackboard for an object with the specified objectId.
	 *
	 * @param objectId The objectId of the object.
	 *
	 * @return The document with the specified ObjectId.
	 */
	mongo::BSONObj findDocumentById(mongo::OID objectId);

	/**
	 * Queries the database for all documents matching the given query.
	 *
	 * @param queryAsJSON The query that is used to determine which objects will be returned.
	 * @param results Reference to a std::vector<mongo::BSONObj> that will be used to store the results.
	 *
	 * @return The amount of documents that have been found.
	 */
	int findDocuments(std::string queryAsJSON, std::vector<mongo::BSONObj> &results);

	/**
	 * Updates all documents matching the query according to the specified update query.
	 *
	 * @param queryAsJSON The query used to determine which documents should be updated.
	 * @param updateQueryAsJSON The query that is used to update the matching documents.
	 */
	void updateDocuments(std::string queryAsJSON, std::string updateQueryAsJSON);

private:
	/**
	 * @var std::string host
	 * Hostname of the server.
	 */
	std::string host;

	/**
	 * @var std::string database
	 * The name of the database
	 **/
	std::string database;

	/**
	 * @var std::string collection
	 * The name of the collection
	 **/
	std::string collection;

	/**
	 * @var boost::thread *oplogMonitor
	 * Pointer to OplogMonitor that is used to monitor the oplog.
	 **/
	OplogMonitor *oplogMonitor;
};

}
#endif
