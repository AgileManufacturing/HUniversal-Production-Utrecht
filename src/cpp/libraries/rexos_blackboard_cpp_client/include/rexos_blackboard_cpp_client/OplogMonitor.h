/**
 * @file OplogMonitor.h
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

#ifndef OPLOGMONITOR_H_
#define OPLOGMONITOR_H_

#include <string>
#include <vector>
#include "mongo/client/dbclientinterface.h"
#include "boost/thread.hpp"

namespace Blackboard {

class BlackboardSubscription;

class OplogMonitor {
public:
	OplogMonitor(
			std::string host,
			std::string oplogDBName,
			std::string oplogCollectionName);

	~OplogMonitor();
	void start();
	void interrupt();
	void restart();
	void addSubscription(BlackboardSubscription& sub);
	void removeSubscription(BlackboardSubscription& sub);
	void setNamespace(std::string &database, std::string &collection);
	void setNamespace(std::string omtNamespace);

private:
	void run();
	mongo::Query createOplogQuery();
	int getSkipCount(std::string collectionNamespace);
	long long int currentCursorId;

	std::vector<BlackboardSubscription *> subscriptions;
	mongo::Query query;
	std::string host;
	std::string oplogNamespace;
	std::string omtNamespace;
	boost::mutex subscriptionsMutex;
	boost::thread *currentThread;
};

} /* namespace Blackboard */
#endif /* OPLOGMONITOR_H_ */
