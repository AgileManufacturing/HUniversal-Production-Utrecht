/*
 * TestThingClient.cpp
 *
 *  Created on: May 4, 2012
 *      Author: mb
 */

#include "BlackboardClient/BlackboardClientUtils.h"
#include <AutoKeyStore/AutoKeyStore.h>
#include <iostream>
#include <string>


using namespace nl::hu::lcv::blackboard::data;
using namespace std;
namespace BlackboardClient
{

BlackboardClientUtils::BlackboardClientUtils() {
	// TODO Auto-generated constructor stub
 	
}


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

			if ((he = gethostbyname(pStrHost.c_str())) == NULL)
			{
				perror("Client::Client, gethostbyname");
				return;
			}

			if ((m_iSock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
			{
				perror("Client::Client, socket");
				return;
			}

			m_addrRemote.sin_family		= AF_INET;
			m_addrRemote.sin_port		= htons(m_iPort);
			m_addrRemote.sin_addr		= *((struct in_addr *) he->h_addr);
			memset(&(m_addrRemote.sin_zero), 0, 8);

			if (connect(m_iSock, (struct sockaddr *) &m_addrRemote, sizeof(struct sockaddr)) == -1)
			{
				perror("Client::Client, connect m_iSock");
				return;
			}

			string Str;
			Str = postItBox->SerializeAsString();

			const size_t len = Str.length();
			if (send(m_iSock, Str.data(), len, 0) == -1)
			{
				perror("Client::SendString, send");
			}
			int a[1];
			a[0] = -1;

			send(m_iSock, a, 1, 0);

}


PostItBox * BlackboardClientUtils::readFromBlackboard(PostItBox * postItBox)
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

				if ((he = gethostbyname(pStrHost.c_str())) == NULL)
				{
					perror("Client::Client, gethostbyname");

				}

				if ((m_iSock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
				{
					perror("Client::Client, socket");

				}

				m_addrRemote.sin_family		= AF_INET;
				m_addrRemote.sin_port		= htons(m_iPort);
				m_addrRemote.sin_addr		= *((struct in_addr *) he->h_addr);
				memset(&(m_addrRemote.sin_zero), 0, 8);

				if (connect(m_iSock, (struct sockaddr *) &m_addrRemote, sizeof(struct sockaddr)) == -1)
				{
					perror("Client::Client, connect m_iSock");

				}

				string Str;
				Str = postItBox->SerializeAsString();

				const size_t len = Str.length();
				if (send(m_iSock, Str.data(), len, 0) == -1)
				{
					perror("Client::SendString, send");
				}
				int a[1];
				a[0] = -1;

				send(m_iSock, a, 1, 0);

				
				int count = 0;
				std::vector<char>  buffer;

				char pTemp[2];
				while((count != -1) && (count = recv(m_iSock, pTemp, 1,0)) > 0)
				{
					if(count == -1)
						break;
					else
						{
							buffer.push_back(pTemp[0]);
						}
						//remainingSize -= count;
				}
				
				postItBox->ParseFromArray(buffer.data(), buffer.size());
				return postItBox;


}



BlackboardClientUtils::~BlackboardClientUtils() {
	// TODO Auto-generated destructor stub
}
}
