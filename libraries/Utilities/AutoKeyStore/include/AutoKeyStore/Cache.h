//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        LibAutoKeyStore
// File:           Cache.h	  
// Description:    The Cache class is a library class that implements a caching 
//			mechanism. 
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
#ifndef CACHE_H
#define	CACHE_H

#include <ctime>
#include <list>
#include <string>
#include <map>

using namespace std;

//5 minutes
#define MAX_CACHE_TIME (5*60)

/**
 * The CacheObject is a wrapper around the data to be stored, implementing 
 * controls for the caching mechanism. 
*/
class CacheObject
{
public:
    CacheObject(string value = "");

    bool isOutdated() const;
    string getValue() const;
    void update(string value);
private:
    /**
    * 
    * Data value of this CacheObject. 
    */
    string value;
    /**
    * The moment in time this object was cached.
    */
    long timeCached;
};

/**
 * The Cache class is used for caching values for a limited amount of time to improve performance. 
 * This class is used to improves the performance of repetitively asking for data from the 
 * AutoKeyStore server. 
 * @author Pascal
 */
class Cache
{
public:
    bool checkCache(const list<string>& keys, map<string, string>& returnValues);
    void updateCache(map<string, CacheObject>& KeyValues);
    void updateCacheEntry(const string& key, const string& value);
private:
    /**
     * A Map is used as an associative array using strings as indexes. 
     */
    map<string, CacheObject> cache;
};



#endif	/* CACHE_H */

