/*
 * TestThingClient.h
 *
 *  Created on: May 4, 2012
 *      Author: mb
 */

#ifndef BLACKBOARDCLIENTUTILS_H_
#define BLACKBOARDCLIENTUTILS_H_

#include "BlackboardClient/PostIt.pb.h"
#include <AutoKeyStore/AutoKeyStore.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>


using namespace nl::hu::lcv::blackboard::data;

namespace BlackboardClient
{
	class BlackboardClientUtils {
	private:
		AutoKeyStore aks;
	public:
		BlackboardClientUtils();

		void writeToBlackboard(PostItBox * postItBox);
		PostItBox * readFromBlackboard(PostItBox * postItBox);
		virtual ~BlackboardClientUtils();
	};
}
#endif /* BLACKBOARDCLIENTUTILS_H_ */
