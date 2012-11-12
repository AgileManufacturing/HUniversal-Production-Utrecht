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

BlackboardCppClient::BlackboardCppClient(const std::string &hostname) {
  try {
    std::cout << "connected to database" << std::endl;
  	connection.connect(hostname);
  } catch( const mongo::DBException &e ) {
    std::cout << "caught " << e.what() << std::endl;
  }
}


BlackboardCppClient::BlackboardCppClient(const std::string &hostname, int port) {
  try {
    std::cout << "connected to database" << std::endl;
  	connection.connect(mongo::HostAndPort(hostname, port));
  } catch( const mongo::DBException &e ) {
    std::cout << "caught " << e.what() << std::endl;
  }	
}

void BlackboardCppClient::setDatabase(const std::string &db) {
	database = db;
}

void BlackboardCppClient::setCollection(const std::string &col) {
	collection = col;
}

void BlackboardCppClient::subscribe(const std::string &topic) {
	if(collection.empty()) {

	} else if(database.empty()) {

	}
	subscriptions.insert( std::pair<std::string, mongo::BSONObj>(topic, BSON("topic" << topic)) );	
	if(subscriptions.size() == 1) {
		readMessageThread = new boost::thread(run, this);
	}
}

void BlackboardCppClient::unsubscribe(const std::string &topic) { 
	readMessageThread.interrup();
}


void BlackboardCppClient::run() {
	// Create namespace string
	/*std::string namespace(database);
	namespace.append(".");
	namespace.append(collection);
	std::cout << bla << std::endl;
	
	Query query = Query().sort("$natural");
	std::string id = "";
	std::string operation = "";

	while(true) {
		auto_ptr<DBClientCursor> tailedCursor = connection.query(namespace.c_str(), query, 0, 0, 0, 
			QueryOption_CursorTailable | QueryOption_AwaitData );
		while(true) {
			if(!tailedCursor->more()) {
				if(tailedCursor->isDead()) {
					break;
				}
				continue;
			}
			BSONObj addedObject = tailedCursor->next();
			if(addedObject.hasField("o")) {
				id = addedObject.getObjectField("o").getStringField("_id");
			}
			if(addedObject.hasField("o2") && id == null || id.empty()) {
				id = addedObject.getObjectField("o2").getStringField("_id");
			}
			operation = addedObject.getStringField("op");
			mongo::BSONObj messageCheckObject(BSON("_id" << mongo::OID(id) << "$or" << )); 

			lastId = object["_id"];
		}
	}*/
}

