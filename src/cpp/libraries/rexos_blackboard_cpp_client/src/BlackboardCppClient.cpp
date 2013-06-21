/**
 * @file BlackboardCppClient.cpp
 * @brief Represents a blackboard connection.
 * @date Created: 2012-11-12
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

#include <iostream>
#include "mongo/client/connpool.h"
#include "rexos_blackboard_cpp_client/BlackboardCppClient.h"
#include "rexos_blackboard_cpp_client/InvalidDBNamespaceException.h"
#include "rexos_blackboard_cpp_client/OplogMonitor.h"
#include "rexos_blackboard_cpp_client/BlackboardSubscription.h"

using namespace Blackboard;

BlackboardCppClient::BlackboardCppClient(const std::string &hostname,
		const std::string db, const std::string coll) :
		host(hostname), database(db), collection(coll), oplogMonitor(NULL)
{

}

BlackboardCppClient::BlackboardCppClient(const std::string &hostname, int port,
		const std::string db, const std::string coll) :
		host(mongo::HostAndPort(hostname, port).toString()),database(db), collection(coll), oplogMonitor(NULL)
{

}

BlackboardCppClient::~BlackboardCppClient(){
	if (oplogMonitor != NULL) {
		delete oplogMonitor;
	}
}

void BlackboardCppClient::setDatabase(const std::string &db){
	if (db.empty()) {
		throw InvalidDBNamespaceException("Database name may not be empty.");
	}
	database = db;
	collection = "";

	if (oplogMonitor != NULL) {
		oplogMonitor->setNamespace(database, collection);
	}
}

void BlackboardCppClient::setCollection(const std::string &col){
	if (col.empty()) {
		throw InvalidDBNamespaceException("Collection name may not be empty.");
	}
	collection = col;

	if (oplogMonitor != NULL) {
		oplogMonitor->setNamespace(database, collection);
	}
}

void BlackboardCppClient::subscribe(BlackboardSubscription &sub){
	if (database.empty() || collection.empty()) {
		throw InvalidDBNamespaceException("Database and collection names may not be empty.");
	}

	if (oplogMonitor == NULL) {
		oplogMonitor = new OplogMonitor(host, "local", "oplog.rs");
		oplogMonitor->setNamespace(database, collection);
	}

	oplogMonitor->addSubscription(sub);
}

void BlackboardCppClient::unsubscribe(BlackboardSubscription &sub){
	if (oplogMonitor != NULL) {
		oplogMonitor->removeSubscription(sub);
	}
}

void BlackboardCppClient::insertDocument(std::string json){
	mongo::ScopedDbConnection* connection = mongo::ScopedDbConnection::getScopedDbConnection(host);
	std::string name = database;
	name.append(".");
	name.append(collection);
	mongo::BSONObj bobj = mongo::fromjson(json);
	(*connection)->insert(name, bobj);
	connection->done();
	delete connection;
}

void BlackboardCppClient::removeDocuments(std::string queryAsJSON) {
	mongo::ScopedDbConnection* connection = mongo::ScopedDbConnection::getScopedDbConnection(host);
	std::string dbNamespace = database;
	dbNamespace.append(".");
	dbNamespace.append(collection);
	mongo::BSONObj query = mongo::fromjson(queryAsJSON);
	(*connection)->remove(dbNamespace, query, false);
	connection->done();
	delete connection;
}

mongo::BSONObj BlackboardCppClient::findDocumentById(mongo::OID objectId) {
	mongo::ScopedDbConnection* connection = mongo::ScopedDbConnection::getScopedDbConnection(host);
	std::string dbNamespace = database;
	dbNamespace.append(".");
	dbNamespace.append(collection);
	mongo::Query query = QUERY("_id" << objectId);
	mongo::BSONObj document = (*connection)->findOne(dbNamespace, query);
	connection->done();
	delete connection;
	return document;
}


int BlackboardCppClient::findDocuments(std::string queryAsJSON, std::vector<mongo::BSONObj> &results) {
	mongo::ScopedDbConnection* connection = mongo::ScopedDbConnection::getScopedDbConnection(host);
	std::string dbNamespace = database;
	dbNamespace.append(".");
	dbNamespace.append(collection);
	mongo::Query query(mongo::fromjson(queryAsJSON));
	std::auto_ptr<mongo::DBClientCursor> cursor = (*connection)->query(dbNamespace, query);

	int resultCount = 0;
	try {
		while (cursor->more()) {
			mongo::BSONObj obj = cursor->nextSafe();
			results.push_back(obj);
			resultCount++;
		}
	} catch (mongo::AssertionException &ex) {
		// Error occurred at remote server, there will be no more objects.
	}

	connection->done();
	delete connection;
	return resultCount;
}


void BlackboardCppClient::updateDocuments(std::string queryAsJSON, std::string updateQueryAsJSON) {
	mongo::ScopedDbConnection* connection = mongo::ScopedDbConnection::getScopedDbConnection(host);
	std::string dbNamespace = database;
	dbNamespace.append(".");
	dbNamespace.append(collection);
	mongo::Query query(mongo::fromjson(queryAsJSON));
	mongo::BSONObj updateQuery = mongo::fromjson(updateQueryAsJSON);
	(*connection)->update(dbNamespace, query, updateQuery, false, true);
	connection->done();
	delete connection;
}