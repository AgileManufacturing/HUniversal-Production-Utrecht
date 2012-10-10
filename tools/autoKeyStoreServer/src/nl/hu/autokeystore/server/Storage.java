//******************************************************************************
//
//                 Multi-agentbased production scheduling
//
//******************************************************************************
// Project:        AutoKeyStoreServer
// File:           Storage.java	  
// Description:    AutoKeyStore is a server that will multicast it's existence 
//                  over the LAN and offers a service for retrieving values.  
//                  This server needs a storage object that stores the values. 
// Author:         Pascal Muller
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of AutoKeyStore.
//
// AutoKeyStoreServer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// AutoKeyStoreServer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with AutoKeyStoreServer.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
package nl.hu.autokeystore.server;

import java.io.FileReader;
import java.io.FileWriter;
import java.util.Properties;

/**
 * The Storage class is responsible for storing and retrieving key-value-pairs. 
 * @author pascal
 */
public class Storage
{
    //I opted to use Properties instad of doing this myself using a HashTable, 
    // because this makes loading and saving to a file so much easier. 
    private Properties props;

    /**
     * Constructor.
     */
    public Storage()
    {
        props = new Properties();
    }

    /**
     * Constructor with a filename as a parameter. The contents of the file are 
     * loaded into the storage. 
     * @param fileName The file
     */
    public Storage(String fileName)
    {
        this();
        try
        {
            props.load(new FileReader(fileName));
        }
        catch (Exception ex)
        {
            System.out.println("Warning: Properties file could not be read. Starting with an empty database");
        }
    }

    /**
     * Insert either updates or adds the value of a key depending on whether the 
     * key exists or not. 
     * @param key The key.
     * @param value The value for the key. 
     */
    public void insert(String key, String value)
    {
        props.setProperty(key, value);
    }

    /**
     * Get the value of a key. 
     * @param key The key.
     * @return The value of the key is returned. If the key does not exist, 
     * null is returned. 
     */
    public String get(String key)
    {
        return props.getProperty(key);
    }

    /**
     * Function to check whether the value for a certain key is stored. 
     * @param key The key. 
     * @return True if the key exists in the storage, false otherwise. 
     */
    public boolean exists(String key)
    {
        return props.containsKey(key);
    }

    /**
     * Writes the current contents of the storage to a file. 
     * @param fileName The file.
     */
    public void dump(String fileName)
    {
        try
        {
            props.store(new FileWriter(fileName), "These values can be loaded in the key value store on startup");
        }
        catch (Exception ex)
        {
            System.out.println("Warning: Properties file could not be written");
        }
    }
}
