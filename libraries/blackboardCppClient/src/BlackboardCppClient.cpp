/**
 * @file BlackboardCppClient.cpp
 * @brief The cpp client for the blackboard
 * @date Created: 2012-11-12
 *
 * @author Dennis Koole
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2012, HU University of Applied Sciences Utrecht.
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
#include <blackboardCppClient/BlackboardCppClient.h>
#include <Utilities/Utilities.h>
#include <memory>
#include <unistd.h>

/**
 * Constructor for the BlackboardCppClient
 *
 * @param hostname the name of the host where the mongo database can be found
 **/
BlackboardCppClient::BlackboardCppClient(const std::string &hostname): database(), collection() {
  try {
    connection.connect(hostname);
    std::cout << "connected to database" << std::endl;
  } catch( const mongo::DBException &e ) {
    std::cout << "caught " << e.what() << std::endl;
  }
}

/**
 * Constructor for the BlackboardCppClient
 *
 * @param hostname the name of the host where the mongo database can be found
 **/
BlackboardCppClient::BlackboardCppClient(const std::string &hostname, int port): database(), collection() {
  try {
    std::cout << "connected to database" << std::endl;
  	connection.connect(mongo::HostAndPort(hostname, port));
  } catch( const mongo::DBException &e ) {
    std::cout << "caught " << e.what() << std::endl;
  }	
}

/**
 * destructor for the BlackboardCppClient
 *
 **/
BlackboardCppClient::~BlackboardCppClient() {
	readMessageThread->interrupt();
	delete readMessageThread;
}

/**
 * Set the name of the database to use
 * 
 * @param db the name of the database to use
 **/
void BlackboardCppClient::setDatabase(const std::string &db) {
	database = db;
}

/**
 * Set the name of the collection to use
 *
 * @param col the name of the collection to use
 **/
void BlackboardCppClient::setCollection(const std::string &col) {
	collection = col;
}

/**
 * Subscribe to a blackboard topic
 *
 * Subscribe to a blackboard topic. When the first subscription is added, 
 * a thread will be started to handle the messages from the blackboard 
 *
 * @param topic the name of the topic to subscribe to
 **/
void BlackboardCppClient::subscribe(const std::string &topic) {
	if(collection.empty()) {
		std::cerr << "Collection is empty" << std::endl;
	}
	if(database.empty()) {
		std::cerr << "Database is empty" << std::endl;
	}
	subscriptions.insert( std::pair<std::string, mongo::BSONObj>(topic, BSON("topic" << topic)) );
	// Start thread to read from blackboard	
	if(subscriptions.size() == 1) {
		readMessageThread = new boost::thread(run, this);	
	}
}

/**
 * Unsubscribe from a blackboard topic
 *
 * Unsubscribe from a blackboard topic, 
 * when there are no subscriptions to topics anymore, the thread
 * that reads the messages from the blackboard will be interrupted
 *
 * @param topic the name of the topic 
 **/
void BlackboardCppClient::unsubscribe(const std::string &topic) { 
	subscriptions.erase(topic);
	if(subscriptions.size() == 0) {
		readMessageThread->interrupt();	
	}
}

/**
 * Create a vector of all the values in a map
 *
 * @param m The map with key type K and value type V
 * @param v The vector with that contains objects of the same type V as the map values
 *
 **/
template<typename K, typename V>
void createVectorOfMapValues(const std::map<K, V> &m, std::vector<V> &v) {
    for(typename std::map<K, V>::iterator it = m.begin(); it != m.end(); it++) {
        v.push_back(it->second);
    }
}


void BlackboardCppClient::run(BlackboardCppClient* client) {
	/*while(true) {
		std::cout << "Hello from thread! " <<std::endl;
		sleep(5);	
	}*/
	// Create namespace string
	std::string name = client->database;
	name.append(".");
	name.append(client->collection);
	//mongo::BSONElement lastId = mongo::maxKey.firstElement();
	mongo::Query where = QUERY("ns" << name);
	std::cout << "where: " << where.toString() << std::endl;
	
	//mongo::Query query = mongo::Query().sort("$natural");
	std::string id = "";
	std::string operation = "";

	while(true) {
		//std::cout << "Begin of while" << std::endl;
		std::auto_ptr<mongo::DBClientCursor> tailedCursor = client->connection.query("local.oplog.rs", where, 0, 0, 0, 
			mongo::QueryOption_CursorTailable | mongo::QueryOption_AwaitData );
		//std::cout << "Tailable cursor size: " << tailedCursor->itcount();
		while(true) {
			//std::cout << "Begin of while2" << std::endl;
			if(!tailedCursor->more()) {
				if(tailedCursor->isDead()) {
					break;
				}
				continue;
			}
			//std::cout << "Before addedObject" << std::endl;
			mongo::BSONObj addedObject = tailedCursor->next();
			mongo::BSONElement out;
			
			std::cout << "Added object " << addedObject.toString() << std::endl;
			if(addedObject.hasField("o")) {
				addedObject.getObjectField("o").getObjectID(out);
				id = out.OID().toString();
			}
			if(addedObject.hasField("o2") && id.empty()) {
				addedObject.getObjectField("o2").getObjectID(out);
				id = out.OID().toString();
			}
			std::cout << "After ifjes" << std::endl;
			operation = addedObject.getStringField("op");
			std::vector<mongo::BSONObj> values;
		    for(std::map<std::string, mongo::BSONObj>::iterator it = client->subscriptions.begin(); it != client->subscriptions.end(); it++) {
        		values.push_back(it->second);
    		}
    		//std::cout << "After create vector from map" << std::endl;
			//client->createVectorOfMapValues <std::string, mongo::BSONObj> (client->subscriptions, values);
			std::cout << "Id: " << id << std::endl;
			std::cout << "Operation: " << operation << std::endl;
			mongo::BSONObj messageCheckObject(BSON("_id" << mongo::OID(id) << "$or" << values)); 
			mongo::BSONObj message = client->connection.findOne(name, messageCheckObject);
			if(operation.compare("i") == 0) {
				std::cout << "Message added" << std::endl;
			} else if(operation.compare("u") == 0) {
				std::cout << "Message updated" << std::endl;
			} else if(operation.compare("r") == 0) {
				std::cout << "Message removed" << std::endl;
			}
		}
		//std::cout << " Your query is dead!" << std::endl;
	}
}

