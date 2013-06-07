/**
 * @file OplogMonitorThread.cpp
 * @brief 
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

#include <iostream>

#include "rexos_blackboard_cpp_client/OplogMonitorThread.h"
#include "mongo/client/dbclientinterface.h"
#include "boost/bind.hpp"
#include "rexos_blackboard_cpp_client/OplogEntry.h"
#include "rexos_blackboard_cpp_client/BlackboardSubscriber.h"
#include "rexos_blackboard_cpp_client/BlackboardSubscription.h"

namespace Blackboard {

OplogMonitorThread::OplogMonitorThread(
		mongo::DBClientConnection& connection,
		std::string oplogDBName,
		std::string oplogCollectionName) :
				connection(connection),
				oplogDBName(oplogDBName),
				oplogCollectionName(oplogCollectionName),
				thread(NULL)
{
}

void OplogMonitorThread::start()
{
	thread = new boost::thread(boost::bind(&OplogMonitorThread::run, this));
	thread->join();
}

void OplogMonitorThread::interrupt()
{
	thread->interrupt();
}

mongo::Query OplogMonitorThread::createOplogQuery()
{
	mongo::Query query;
	mongo::BSONArrayBuilder orArray;

	for (std::vector<BlackboardSubscription *>::iterator iter = subscriptions.begin() ; iter != subscriptions.end() ; iter++) {
		if ((*iter)->getQuery(&query)) {
			orArray.append(query);
		}
	}

	return QUERY("$or" << orArray.arr());
}

int OplogMonitorThread::getSkipCount()
{
	mongo::BSONObj ret;
	mongo::BSONObj bsonQry = BSON("collStats" << oplogCollectionName);
	connection.runCommand(oplogDBName, bsonQry, ret);
	return ret.getIntField("count");
}

void OplogMonitorThread::run()
{
	bool interrupted = false;

	// Creating tailable cursor and skipping documents
	std::auto_ptr<mongo::DBClientCursor> tailedCursor = connection.query(
			oplogDBName.append(".").append(oplogCollectionName),
			query,
			0,
			getSkipCount(),
			NULL,
			mongo::QueryOption_CursorTailable | mongo::QueryOption_AwaitData,
			0);

	while(!interrupted){
		try {
			while(tailedCursor->more()){
				const OplogEntry oplogEntry(tailedCursor->next());
				std::cout << "OMT: " << oplogEntry.toString() << std::endl;
				for (std::vector<BlackboardSubscription *>::iterator iter = subscriptions.begin() ; iter != subscriptions.end() ; iter++) {
					if ((*iter)->matchesWithEntry(oplogEntry)) {
						(*iter)->getSubscriber().blackboardReadCallback(oplogEntry.toString());
					}
				}
			}
		} catch (boost::thread_interrupted& e) {
			interrupted = true;
		}
	}
}

void OplogMonitorThread::addSubscription(BlackboardSubscription& sub)
{
	subscriptions.push_back(&sub);
}

} /* namespace Blackboard */
