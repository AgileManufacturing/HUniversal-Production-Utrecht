//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        LibAutoKeyStore
// File:           Cache.cpp	  
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


