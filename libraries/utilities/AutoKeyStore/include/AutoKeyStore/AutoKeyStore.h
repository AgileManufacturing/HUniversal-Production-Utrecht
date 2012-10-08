/**
* @file AutoKeyStore.h
* @brief Can be used by Java programs to access values of keys on the AutoKeyStore configuration server.
*
* @author Pascal Muller
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

#ifndef AUTOKEYSTORE_H
#define	AUTOKEYSTORE_H

#include "AutoKeyStore/Cache.h"

#include <iostream>
#include <list>
#include <map>
#include <stdexcept>

#include <netinet/in.h>

using namespace std;

/**
 * The AutoKeyStoreException is thrown when autoconfiguring fails. 
 */
class AutoKeyStoreException : public runtime_error
{
public:

    AutoKeyStoreException(const string &err) : runtime_error(err)
    {  }
};

class AutoKeyStore
{
public:
    AutoKeyStore();

    void getValue(const string& key, string& output) throw (AutoKeyStoreException);
    void getValues(const list<string>& keys, map<string, string>& output) throw (AutoKeyStoreException);

private:
    string ip;
    int port;
    Cache cache;
    struct sockaddr_in multicast_addr;
    struct sockaddr_in keyval_addr;

    bool checkServer();
    bool discoverServer();
    int createMulticastSocket();
    int createSocket();
    bool parseRL(string& ip, int& port, const char* rlStr);
};

#endif	/* AUTOKEYSTORE_H */

