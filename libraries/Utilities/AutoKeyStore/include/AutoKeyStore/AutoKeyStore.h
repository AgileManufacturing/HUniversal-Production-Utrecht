//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        LibAutoKeyStore
// File:           AutoKeyStore.h	  
// Description:    The AutoKeyStore class is a library class that can be used by 
//                  Java programs to access values of keys on the AutoKeyStore 
//                  configuration server. 
// Author:         Pascal Muller
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of LibAutoKeyStore.
//
// LibAutoKeyStore is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// LibAutoKeyStore is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with LibAutoKeyStore.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
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

