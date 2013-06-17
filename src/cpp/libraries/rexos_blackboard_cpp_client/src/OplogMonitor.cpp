/**
 * @file OplogMonitor.cpp
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

#include "rexos_blackboard_cpp_client/OplogMonitor.h"
#include "mongo/client/connpool.h"
#include "boost/bind.hpp"
#include "rexos_blackboard_cpp_client/OplogEntry.h"
#include "rexos_blackboard_cpp_client/BlackboardSubscriber.h"
#include "rexos_blackboard_cpp_client/BlackboardSubscription.h"

namespace Blackboard {

OplogMonitor::OplogMonitor(
		std::string host,
		std::string oplogDBName,
		std::string oplogCollectionName) :
				host(host),
				oplogNamespace()
{
	std::stringstream nsStream;
	nsStream << oplogDBName << "." << oplogCollectionName;
	oplogNamespace = nsStream.str();
}

OplogMonitor::~OplogMonitor() {
	interrupt();
}

void OplogMonitor::start()
{
	if (subscriptions.size() > 0) {
		currentThread = new boost::thread(boost::bind(&OplogMonitor::run, this));
	}
}

void OplogMonitor::interrupt()
{
	if (currentThread != NULL) {
		currentThread->interrupt();

		mongo::ScopedDbConnection* connection = mongo::ScopedDbConnection::getScopedDbConnection(host);
		(*connection)->killCursor(currentCursorId);
		connection->done();
		delete connection;
		delete currentThread;
	}
}

void OplogMonitor::restart() {
	interrupt();
	start();
}

mongo::Query OplogMonitor::createOplogQuery()
{
	mongo::Query query;
	mongo::BSONArrayBuilder orArray;

	subscriptionsMutex.lock();
	for (std::vector<BlackboardSubscription *>::iterator iter = subscriptions.begin() ; iter != subscriptions.end() ; iter++) {
		if ((*iter)->getQuery(&query)) {
			orArray.append(query.obj);
		}
	}
	subscriptionsMutex.unlock();

	if (!omtNamespace.empty()) {
		mongo::BSONArrayBuilder andArray;
		andArray.append(BSON(OplogEntry::NAMESPACE_FIELD << omtNamespace));
		andArray.append(BSON("$or" << orArray.arr()));
		query = QUERY("$and" << andArray.arr());
	} else {
		query = QUERY("$or" << orArray.arr());
	}

	std::cout << "Using query: " << query << std::endl;
	return query;
}

void OplogMonitor::run()
{
	mongo::ScopedDbConnection* connection = mongo::ScopedDbConnection::getScopedDbConnection(host);

	std::auto_ptr<mongo::DBClientCursor> tailedCursor = (*connection)->query(
			oplogNamespace,
			createOplogQuery(),
			0,
			(*connection)->count(oplogNamespace),
			NULL,
			mongo::QueryOption_CursorTailable | mongo::QueryOption_AwaitData,
			0);

	currentCursorId = tailedCursor->getCursorId();
	try {
		while(!tailedCursor->isDead()){
			while (tailedCursor->more()) {
				const OplogEntry oplogEntry(tailedCursor->nextSafe());
				std::cout << oplogEntry.toString() << std::endl;
				subscriptionsMutex.lock();
				for (std::vector<BlackboardSubscription *>::iterator iter = subscriptions.begin() ; iter != subscriptions.end() ; iter++) {
					if ((*iter)->matchesWithEntry(oplogEntry)) {
						(*iter)->getSubscriber().onMessage(**iter, oplogEntry.toString());
					}
				}
				subscriptionsMutex.unlock();
				boost::this_thread::interruption_point();
			}
			boost::this_thread::interruption_point();
		}
		std::cout << "Cursor is dead." << std::endl;
	} catch (boost::thread_interrupted& e) {
		// Thread has been interrupted, work is done.
		std::cout << "Interrupted" << std::endl;
	} catch (mongo::AssertionException& ex) {
		std::cout << "Assertion exception: " << ex.what() << std::endl;
	}

	connection->done();
	delete connection;

	std::cout << "run stopped" << std::endl;
}

void OplogMonitor::addSubscription(BlackboardSubscription& sub)
{
	subscriptionsMutex.lock();
	subscriptions.push_back(&sub);
	subscriptionsMutex.unlock();
	restart();
}

void OplogMonitor::removeSubscription(BlackboardSubscription& sub)
{
	subscriptionsMutex.lock();
	subscriptions.erase(std::remove(subscriptions.begin(), subscriptions.end(), &sub), subscriptions.end());
	subscriptionsMutex.unlock();
	restart();
}

void OplogMonitor::setNamespace(std::string &database, std::string &collection)
{
	std::stringstream nsStream;
	nsStream << database << "." << collection;
	setNamespace(nsStream.str());
}

void OplogMonitor::setNamespace(std::string omtNamespace)
{
	this->omtNamespace = omtNamespace;
	interrupt();
	subscriptionsMutex.lock();
	subscriptions.clear();
	subscriptionsMutex.unlock();
}

} /* namespace Blackboard */
