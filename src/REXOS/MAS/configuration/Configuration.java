/**
 * @file src/REXOS/MAS/configuration/Configuration.java
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
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Properties;

public class Configuration {
	//private static String propertiesFilePath = System.getenv("PROPERTIESPATH");
	private static String propertiesFilePath = "src/REXOS/MAS/configuration/";
	
	public static String getProperty(ConfigurationFiles file, String key)
	{
		if(file != ConfigurationFiles.EQUIPLET_DB_PROPERTIES){
			return getProperty(file, key, null);
		} else {
			System.err.println("Need a equiplet name to read equiplet db properties");
			return "";
		}
	}
	
	
	public static String getProperty(ConfigurationFiles File, String key, String equipletName)
	{
		Properties p = new Properties();
		try { 
			switch(File)
			{
				case EQUIPLET_DB_PROPERTIES:
					p.load(new FileInputStream(propertiesFilePath + File.getFileName()));
					return p.getProperty(equipletName + key);
					
				default:
					if(File != null){
						p.load(new FileInputStream(propertiesFilePath + File.getFileName()));
						return p.getProperty(key);
					}
					System.out.println("Property file not known.");
					break;
			}
		}catch(NullPointerException e) {
			System.err.println("Property doesnt exist: " + key + " in file " + File.toString());
		} catch (FileNotFoundException e) {
			System.err.println(
					"File doesnt exist: " + File.toString() + 
					", using path: " + propertiesFilePath + 
					", using working directory: " + System.getProperty("user.dir"));
		} catch (IOException e) {
			System.err.println(e);
		}
		return "";
	}
	
	public static int getPropertyInt(ConfigurationFiles File, String key)
	{
		if(File != ConfigurationFiles.EQUIPLET_DB_PROPERTIES){
			return getPropertyInt(File, key, null);
		} else {
			System.err.println("Need a equiplet name to read equiplet db properties");
			return -1;
		}
	}
	
	public static int getPropertyInt(ConfigurationFiles File, String key, String EquipletName)
	{
		try {
			return Integer.parseInt(getProperty(File, key, EquipletName));
		} catch(NumberFormatException e){
			System.err.println("Cant parse integer in configurationfile: " + File.getFileName());
			return -1;
		}
	}
	
}
