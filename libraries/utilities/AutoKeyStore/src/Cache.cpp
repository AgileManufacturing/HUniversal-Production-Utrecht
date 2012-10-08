/**
* @file Cache.cpp
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

#include "AutoKeyStore/Cache.h"

#include <iostream>

/**
 * Returns true if all keys were found. The calling code then knows it 
 * doesn't have to do anything else for these keys. The values from the 
 * cache are returned in returnValues
 * 
 * @param keys Keys to check
 * @param returnValues Values from cache are returned in returnValues
 * @return Returns true if all keys were found in cache.
*/
bool Cache::checkCache(const list<string>& keys, map<string, string>& returnValues)
{
    bool everythingFound = true; //Innocent untill proven guilty
    std::list<string>::const_iterator keyIterator;
    std::map<string, CacheObject>::const_iterator mapIterator;
    for (keyIterator = keys.begin(); keyIterator != keys.end(); ++keyIterator)
    {
	string s(*keyIterator);
	mapIterator = cache.find(s);
	if (mapIterator != cache.end() && !(mapIterator->second).isOutdated()) //Item found in cache?
	{
	    //cout << "cacheHit: " << s << " = " << (mapIterator->second).getValue() << std::endl;
	    returnValues[*keyIterator] = string((mapIterator->second).getValue());
	}
	else
	{
	    //cout << "cacheMiss: " << *keyIterator << std::endl;
	    everythingFound = false;
	}

    }
    return everythingFound;
}


/**
 * Update the value of a key in the cache. 
 * @param key The key to be updated.
 * @param value The value. 
*/
void Cache::updateCacheEntry(const string& key, const string& value)
{
    //Check if it already exists
    map<string, CacheObject>::iterator it;
    if ((it = cache.find(key)) != cache.end())
    {
	(it->second).update(value); //update
    }
    else
    {
	cache[key] = CacheObject(value); //add
    }
}

/**
 * Update the value of multiple keys in the cache. 
 * @param KeyValues An map of keys to be updated and their values. 
*/
void Cache::updateCache(map<string, CacheObject>& KeyValues)
{
    std::map<string, CacheObject>::iterator it;
    //For all keys (and their values), call updateCacheEntry(key, value);
    for (it = KeyValues.begin(); it != KeyValues.end(); ++it)
    {
	this->updateCacheEntry(it->first, (it->second).getValue());
    }
}

/**
 * Constructor sets timeCached to the current time and sets the value of this object. 
 * @param value The value to be hold in this CacheObject. 
*/
CacheObject::CacheObject(string value)
{
    this->value = value;
    this->timeCached = time(NULL);
}

/**
 * Checks if this object is outdated. 
 * @return true if this CacheObject was cached longer than MAX_CACHE_TIME. 
*/
bool CacheObject::isOutdated() const
{
    return (time(NULL) - this->timeCached > MAX_CACHE_TIME);
}

/**
 * Returns value stored in this object. 
 * @return value stored
*/
string CacheObject::getValue() const
{
    return this->value;
}

/**
 * Update this object, the timeCached is set to the current time. 
 * @param value The new value. 
*/
void CacheObject::update(string value)
{
    this->value = value;
    this->timeCached = time(NULL);
}


