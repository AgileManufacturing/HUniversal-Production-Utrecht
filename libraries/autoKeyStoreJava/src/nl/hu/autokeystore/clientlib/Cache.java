//******************************************************************************
//
//                 AutoKeyStoreJava
//
//******************************************************************************
// Project:        AutoKeyStoreJava
// File:           Cache.java     
// Description:    ?
// Author:         Pascal Muller
// Notes:          
//
// License:        newBSD
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

package nl.hu.autokeystore.clientlib;

import java.util.HashMap;

/**
 * The Cache class is used for caching values for a limited amount of time to improve performance. 
 * This class is used to improves the performance of repetitively asking for data from the 
 * AutoKeyStore server. 
 * @author Pascal
 */
public class Cache
{
    /**
     * A HashMap is used as an associative array using strings as indexes. 
     */
    HashMap<String, CacheObject> cache;
    
    /**
     * Constructor
     */
    public Cache()
    {
        cache = new HashMap<String, CacheObject>();
    }
    
    /**
     * The CacheObject is a wrapper around the data to be stored, implementing 
     * controls for the caching mechanism. 
     */
    private class CacheObject
    {
        /**
         * The moment in time this object was cached, as returned by System.currentTimeMillis().  
         */
        private long timeCached;
        
        /**
         * 
         * Data value of this CacheObject. 
         */
        private String value;
        
        /**
         * Maximum time CacheObjects are to be kept. 
         */
        private static final long MAX_CACHE_TIME = 5*60*1000; //5 minutes
        
        /**
         * Constructor sets timeCached to the current time and sets the value of this object. 
         * @param value The value to be hold in this CacheObject. 
         */
        public CacheObject(String value)
        {
            this.value = value;
            timeCached = System.currentTimeMillis();
        }
        
        /**
         * Checks if this object is outdated. 
         * @return true if this CacheObject was cached longer than MAX_CACHE_TIME. 
         */
        public boolean isOutdated()
        {
            return (System.currentTimeMillis()-timeCached > MAX_CACHE_TIME);
        }
        
        /**
         * Returns value stored in this object. 
         * @return value stored
         */
        public String getValue()
        {
            return value;
        }
        
        /**
         * Update this object, the timeCached is set to the current time. 
         * @param value The new value. 
         */
        public void update(String value)
        {
            this.value = value;
            timeCached = System.currentTimeMillis();
        }

    }
    
    /**
     * Returns true if all keys were found. The calling code then knows it 
     * doesn't have to do anything else for these keys. The values from the 
     * cache are returned in returnValues
     * 
     * @param keys Keys to check
     * @param returnValues Values from cache are returned in returnValues
     * @return Returns true if all keys were found in cache.
     */
    public boolean checkCache(String[] keys, String[] returnValues)
    {
        //int hitCnt = 0;
        //int missCnt = 0;
        // For all given keys.. 
        for (int i = 0; i < keys.length; i++)
        {
            returnValues[i] = null;
            //.. try to retrieve the value from the cache. 
            CacheObject co = cache.get(keys[i]); 
            if(co != null && !co.isOutdated())
            {
                //If the object is in the cache and still valid, return its value trough returnValues.
                returnValues[i] = co.getValue();
            }
            
            //if(returnValues[i] == null) missCnt++;
            //else hitCnt++;
        }
        //System.out.println("Cache hits: "+hitCnt+" | misses: "+missCnt);
        
        //returnValues is already set, now we should check if all items were found and return. 
        return (!arrayContainsNull(returnValues));
    }
    
    /**
     * Update the value of multiple keys in the cache. 
     * @param keys An array of keys to be updated. 
     * @param vals Values of the keys. 
     */
    public void updateCache(String[] keys, String[] vals)
    {
        for (int i = 0; i < keys.length; i++)
        {            
            updateCacheEntry(keys[i], vals[i]);
        }        
    }
    
    /**
     * Update the value of a key in the cache. 
     * @param key The key to be updated.
     * @param value The value. 
     */
    public void updateCacheEntry(String key, String value)
    {       
        CacheObject co = cache.get(key);
        if(co != null)
        {
            co.update(value);
        }
        else
        {
            cache.put(key, new CacheObject(value));
        }   
    }
    
    /**
     * Checks whether an array has one or more null values. 
     * @param objs
     * @return true if objs has at least one element set to null. 
     */
    private boolean arrayContainsNull(Object[] objs)
    {
        for(int i = 0; i < objs.length; i++)
        {
            if(objs[i] == null)
                return true;
        }
        return false;
    }
    
}


