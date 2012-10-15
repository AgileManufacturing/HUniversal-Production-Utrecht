/**
 * @file BlackboardClientUtils.cpp
 * @brief The C++ blackboard client.
 * @date Created: 2012-05-04
 *
 * @author 1.0 Martijn Beek
 * @author 2.0 Dennis Koole
 *
 * @section LICENSE
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

#include "BlackboardClient/BlackboardClientUtils.h"
#include <AutoKeyStoreClient/AutoKeyStore.h>
#include <iostream>
#include <string>


using namespace nl::hu::lcv::blackboard::data;
using namespace std;
namespace BlackboardClient
{
	void BlackboardClientUtils::writeToBlackboard(PostItBox * postItBox)
	{
		struct hostent*	he = NULL;
		int m_iPort;							// Port I'm listening on
		int m_iSock;							// Socket connection
		struct sockaddr_in	m_addrRemote;		// Connector's address information
		string pStrHost;
		string iPort;

		aks.getValue("blackboard.ip", pStrHost);
		aks.getValue("blackboard.port", iPort);
		m_iPort = atoi(iPort.c_str());
		m_iSock = -1;

		cout << "Client: opening socket to " << pStrHost << " on port = " << m_iPort << "\n";

		if ((he = gethostbyname(pStrHost.c_str())) == NULL) {
			perror("Client::Client, gethostbyname");
			return;
		}

		if ((m_iSock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
			perror("Client::Client, socket");
			return;
		}

		m_addrRemote.sin_family		= AF_INET;
		m_addrRemote.sin_port		= htons(m_iPort);
		m_addrRemote.sin_addr		= *((struct in_addr *) he->h_addr);
		memset(&(m_addrRemote.sin_zero), 0, 8);

		if (connect(m_iSock, (struct sockaddr *) &m_addrRemote, sizeof(struct sockaddr)) == -1) {
			perror("Client::Client, connect m_iSock");
			return;
		}

		string Str;
		Str = postItBox->SerializeAsString();

		const size_t len = Str.length();
		if (send(m_iSock, Str.data(), len, 0) == -1) {
			perror("Client::SendString, send");
		}
		int a[1];
		a[0] = -1;

		send(m_iSock, a, 1, 0);

	}

	PostItBox * BlackboardClientUtils::readFromBlackboard(PostItBox * postItBox) {
		struct hostent*	he = NULL;
		int m_iPort;							// Port I'm listening on
		int m_iSock;							// Socket connection
		struct sockaddr_in	m_addrRemote;		// Connector's address information
		string pStrHost;
		string iPort;
	

		aks.getValue("blackboard.ip", pStrHost);
		aks.getValue("blackboard.port", iPort);
		m_iPort = atoi(iPort.c_str());
		m_iSock = -1;

		cout << "Client: opening socket to " << pStrHost << " on port = " << m_iPort << "\n";

		if ((he = gethostbyname(pStrHost.c_str())) == NULL) {
			perror("Client::Client, gethostbyname");
		}

		if ((m_iSock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
			perror("Client::Client, socket");

		}

		m_addrRemote.sin_family		= AF_INET;
		m_addrRemote.sin_port		= htons(m_iPort);
		m_addrRemote.sin_addr		= *((struct in_addr *) he->h_addr);
		memset(&(m_addrRemote.sin_zero), 0, 8);

		if (connect(m_iSock, (struct sockaddr *) &m_addrRemote, sizeof(struct sockaddr)) == -1) {
			perror("Client::Client, connect m_iSock");
		}

		string Str;
		Str = postItBox->SerializeAsString();

		const size_t len = Str.length();
		if (send(m_iSock, Str.data(), len, 0) == -1) {
			perror("Client::SendString, send");
		}
		int a[1];
		a[0] = -1;

		send(m_iSock, a, 1, 0);

		
		int count = 0;
		std::vector<char>  buffer;

		char pTemp[2];
		while((count != -1) && (count = recv(m_iSock, pTemp, 1,0)) > 0) {
			if(count == -1) {
				break;
			}
			else {
				buffer.push_back(pTemp[0]);
			}
			//remainingSize -= count;
		}
		
		postItBox->ParseFromArray(buffer.data(), buffer.size());
		return postItBox;
	}
}
