//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        LibAutoKeyStore
// File:           AutoKeyStore.cpp	  
// Description:    The AutoKeyStore class is a library class that can be used by 
//                  Java programs to access values of keys on the AutoKeyStore 
//                  configuration server. 
// Author:         Pascal Muller
// Notes:          
//
// License:        newBSD
//
// Copyright © 2012, HU University of Applied Sciences Utrecht
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of the HU University of Applied Sciences Utrecht nor the
// names of its contributors may be used to endorse or promote products
// derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************
#include "AutoKeyStore/AutoKeyStore.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>


#define MAX_IPADRR_LEN 15 //xxx.xxx.xxx.xxx
#define MAX_VALUE_LEN 1024 //Is 1024 bytes enough?

#define MULTICAST_PORT 4446
#define MULTICAST_GROUP "239.255.255.250"
#define MULTICAST_MSGBUFSIZE (5 + MAX_IPADRR_LEN + 1 + 5) //AKSS-<ip>:<port>

/**
 * Constructor for the AutoKeyStore object, which offers the API. 
 * 
 * We should think about offering the option to use this API in a
 * non-blocking fashion. The current implementation only offers 
 * blocking functions.
*/
AutoKeyStore::AutoKeyStore() : ip("NULL"), port(-1)
{
}

/**
 * Gets a single value from the server. For performance reasons, it is 
 * recommended to use getValues() instead of multiple calls to getValue() if 
 * this is possible. 
 * @param key The key. 
 * @param output The value that corresponds with the key is placed in output
 * @throws AutoKeyStoreException When the connection to the server fails for 
 * whatever reason, an exception is thrown. 
 */
void AutoKeyStore::getValue(const string& key, string& output) throw (AutoKeyStoreException)
{
    list<string> keys;
    map<string, string> outs;
    keys.push_back(key);
    
    //We secretly use getValues()
    getValues(keys, outs);
    output = outs[key];
}

/**
 * Retrieves multiple values from the server at once. This function keeps the 
 * connection open when requesting multiple values. It is therefore recommended 
 * to use a single call to this method instead of successive calls to getValue().
 * 
 * @param keys The keys. 
 * @return The values that correspond with the keys. 
 * @throws RuntimeException When the connection to the server fails for 
 * whatever reason, an exception is thrown. 
*/
void AutoKeyStore::getValues(const list<string>& keys, map<string, string>& output) throw (AutoKeyStoreException)
{
    if (this->checkServer() == false)
	throw AutoKeyStoreException("Autconfiguration server does not respond and environment \
	variable AKSS_MASTER_URI is not set (correctly)");

    output.clear(); //Prevent user from interfering with caching, output should always be empty. 
    if (!cache.checkCache(keys, output))
    {
	int fd = this->createSocket();
	// connect: create a connection with the server
	if (fd < 0 || connect(fd, (struct sockaddr*) & this->keyval_addr, sizeof (this->keyval_addr)) < 0)
	    throw AutoKeyStoreException("Connection failed, keystore server seems unreachable. ");

	std::list<string>::const_iterator iterator;
	char buf[MAX_VALUE_LEN];
	int n;
	//For all keys
	for (iterator = keys.begin(); iterator != keys.end(); ++iterator)
	{
	    //Create a copy of the key, so it can me manipulated temporarily. 
	    string s(*iterator); 
	    if (output[s] == "") //If the key was not retrieved from the cache
	    {
		s.append("\n"); //The protocol separates on the newline character
		if (write(fd, s.c_str(), s.length()) < 0)
		    throw AutoKeyStoreException("Writing to keystore server failed. ");

		if ((n = read(fd, &buf, MAX_VALUE_LEN)) < 1)
		    throw AutoKeyStoreException("Reading from keystore server failed. ");
		buf[n - 1] = 0x0; //replace \n (last received byte) with a string ending
		output[*iterator] = string(buf);
		cache.updateCacheEntry(string(*iterator), string(buf));
	    }
	}

	close(fd);
    }
}

/**
 * This function creates the multicast socket and sets the right options. A file 
 * descriptor to the socket is returned. The caller is responsible for cleaning 
 * up this file descriptor. 
 * 
 * @return A negative value is returned in case of an error, else a file descriptor 
 * is returned. 
 */
int AutoKeyStore::createMulticastSocket()
{
    int fd;
    struct ip_mreq mreq;
    // Create what looks like an ordinary UDP socket 
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
	perror("Creating socket failed");
	return -1;
    }

    u_int yes = 1;
    // allow multiple sockets to use the same PORT number
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof (yes)) < 0)
    {
	perror("setsockopt() failed: SO_REUSEADDR");
	return -1;
    }

    // set up destination address
    memset(&(this->multicast_addr), 0, sizeof (this->multicast_addr));
    this->multicast_addr.sin_family = AF_INET;
    this->multicast_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    this->multicast_addr.sin_port = htons(MULTICAST_PORT);

    // bind to receive address
    if (bind(fd, (struct sockaddr *) &(this->multicast_addr), sizeof (this->multicast_addr)) < 0)
    {
	return -1;
    }

    struct timeval timeout;
    timeout.tv_sec = 1; //If nothing is received for a second, reads from the socket will be aborted. 
    timeout.tv_usec = 0;

    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *) &timeout, sizeof (timeout)) < 0)
	perror("setsockopt() failed: SO_RCVTIMEO");


    // use setsockopt() to join a multicast group
    mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof (mreq)) < 0)
    {
	perror("setsockopt() failed: IP_ADD_MEMBERSHIP");
	return -1;
    }
    return fd;
}

/**
 * This function creates the socket that is used for requesting the value of keys. 
 * A file descriptor to the socket is returned. The caller is responsible for 
 * cleaning up this file descriptor. 
 * 
 * @return A negative value is returned in case of an error, else a file descriptor 
 * is returned. 
 */
int AutoKeyStore::createSocket()
{
    int fd;
    struct hostent *server;

    // socket: create the socket
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
	perror("ERROR opening socket");
	return -1;
    }

    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;

    if ((setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *) &timeout, sizeof (timeout)) < 0) ||
	    (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char *) &timeout, sizeof (timeout)) < 0))
    {
	perror("setsockopt() failed: Setting timeout");
    }


    /* gethostbyname: get the server's DNS entry */
    server = gethostbyname(ip.c_str());
    if (server == NULL)
    {
	fprintf(stderr, "ERROR, no such host as %s\n", ip.c_str());
	return -1;
    }

    /* Put the adress in keyval_Addr */
    bzero((char *) &keyval_addr, sizeof (keyval_addr));
    keyval_addr.sin_family = AF_INET;
    bcopy((char *) server->h_addr,
	  (char *) &keyval_addr.sin_addr.s_addr, server->h_length);
    keyval_addr.sin_port = htons(port);

    return fd;
}


/**
 * Checks if the server is already discovered and attempts to discover the server. 
 * 
 * If the server was already discovered, this method immediately stops. 
 * If it isn't, an attempt is made to discover the server. If this fails, the 
 * environment variable AKSS_MASTER_URI is read and used. Should this fail, a 
 * RuntimeException is thrown to inform the user. 
 * 
 * @return 
 */
bool AutoKeyStore::checkServer()
{
    if ((this->ip).compare("NULL") == 0 || this->port == -1)
    {
	if (!this->discoverServer())
	{
	    char* wtf = getenv("AKSS_MASTER_URI");
	    if (wtf != NULL)
	    {
		if (this->parseRL(this->ip, this->port, wtf))
		    return true;
	    }
	    return false;
	}
    }
    return true;
}

/**
 * discoverServer() listens to multicast packets sent by the AutoKeyStore server.
 * @return true if server is successfully discovered and the packet sent by the 
 * server adheres to the correct format. 
*/
bool AutoKeyStore::discoverServer()
{
    int fd, nbytes;
    unsigned int addrlen = sizeof (this->multicast_addr);
    char msgbuf[MULTICAST_MSGBUFSIZE];

    fd = this->createMulticastSocket();
    if (fd == -1)
	return false;
    int tries = 2;
    bool serverFound = false;

    while (tries > 0)
    {
	if ((nbytes = recvfrom(fd, msgbuf, MULTICAST_MSGBUFSIZE, 0, (struct sockaddr *) &(this->multicast_addr), &addrlen)) < 0)
	{
	    tries--; //No packet was received for a second, decrement tries. 
	    continue;
	}
	msgbuf[nbytes] = 0;
	if (strncmp(msgbuf, "AKSS-", 5) == 0)
	{
	    if (this->parseRL(this->ip, this->port, msgbuf + 5)) //server found?
	    {
		serverFound = true;
		break;
	    }
	    tries--; //decrement tries if package is incorrect
	}
    }
    close(fd);
    return serverFound;
}

bool AutoKeyStore::parseRL(string& ip, int& port, const char* rlStr)
{
    //string.c_str() returns a const char* that should not be changed. 
    //I also suspect working on a local copy prevents problems with strtok() 
    //not being thread safe. 
    char* workCopy = new char[strlen(rlStr) * sizeof (char) ];
    strncpy(workCopy, rlStr, strlen(rlStr));

    int lPort = -1;
    string lIp;
    
    bool isOk = false;

    char* tok = strtok(workCopy, ":");
    if (tok != NULL && strlen(tok) < MAX_IPADRR_LEN)
    {
	lIp = string(tok);
    }
    tok = strtok(NULL, ":");
    if (tok != NULL)
    {
	lPort = atoi(tok);
    }
    if (lPort > 0 && lPort < 65536 && lIp.compare("") != 0)
    {
	//Everything is ok, copy local variables to output variables
	ip = lIp;
	port = lPort;
	isOk = true;
    }
    delete[] workCopy;
    return isOk; 
}
