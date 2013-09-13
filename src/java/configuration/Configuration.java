/**
 * @file Configuration.java
 * @brief Configuration file for getting ( and setting? ) configuration properties.
 * @date Created: 18 jun. 2013
 *
 * @author Alexander Streng
 *
 * @section LICENSE
 * License: newBSD
 *
 * Copyright © 2013, HU University of Applied Sciences Utrecht.
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
 * 
 **/
package configuration;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

import rexos.libraries.log.Logger;
import rexos.mas.data.LogLevel;

public class Configuration {
	private static Properties mongoDbProperties;
	private static Properties knowledgeDbProperties;
	private static Properties equipletDbProperties;
	
	public Configuration() {
		mongoDbProperties = new Properties();
		knowledgeDbProperties = new Properties();
		equipletDbProperties = new Properties();
 
    	try {
    		//Get the environment variables and construct the paths
            String mongoDbPropertiesFilePath = System.getenv("PROPERTIESPATH") + "mongo_db.properties";
            String knowledgeDbPropertiesFilePath = System.getenv("PROPERTIESPATH") + "knowledge_db.properties";
            String equipletDbPropertiesFilePath = System.getenv("PROPERTIESPATH") + "equiplet_db.properties";

            //load the properties
    		mongoDbProperties.load(new FileInputStream(mongoDbPropertiesFilePath));
    		knowledgeDbProperties.load(new FileInputStream(knowledgeDbPropertiesFilePath));
    		equipletDbProperties.load(new FileInputStream(equipletDbPropertiesFilePath));
    		
    	} catch (IOException ex) {
    		Logger.log(LogLevel.ERROR, ex);
        }
    }
	
	public static String getProperty(ConfigurationFiles File, String key)
	{
		if(File != ConfigurationFiles.EQUIPLET_DB_PROPERTIES){
			return getProperty(File, key, null);
		} else {
			Logger.log(LogLevel.WARNING, "Need a equiplet name to read equiplet db properties");
			return "";
		}
	}
	
	
	public static String getProperty(ConfigurationFiles File, String key, String EquipletName)
	{
		switch(File)
		{
			case MONGO_DB_PROPERTIES:
				return mongoDbProperties.getProperty(key);
				
			case KNOWLEDGE_DB_PROPERTIES:
				return knowledgeDbProperties.getProperty(key);
				
			case EQUIPLET_DB_PROPERTIES:
				return equipletDbProperties.getProperty(EquipletName + key);
				
			default:
				Logger.log(LogLevel.WARNING, "Property file not know.");
				return "";
		}
	}
	
	public static Boolean isProperty(ConfigurationFiles File, String key)
	{
		switch(File)
		{
			case MONGO_DB_PROPERTIES:
				return mongoDbProperties.containsKey(key);
				
			case KNOWLEDGE_DB_PROPERTIES:
				return knowledgeDbProperties.containsKey(key);
				
			default:
				return false;
		}
	}
	
}
