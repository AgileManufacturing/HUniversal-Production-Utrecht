/**
* @file Cache.h
* @brief The Cache class is a library class that implements a caching mechanism. 
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

