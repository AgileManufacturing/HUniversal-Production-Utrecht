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
package util.configuration;

import java.io.FileInputStream;
import java.io.IOException;

import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

public class Configuration {
	private static final String defaultPropertyName = "default";
	private static final String defaultPropertiesFilePath = "src/REXOS/util/configuration/settings.json";
	
	protected static JSONObject configuration = null;
	
	public static void initialize() {
		String propertiesFilePath = new String();
		try {
			if(System.getenv("PROPERTIESPATH") != null && System.getenv("PROPERTIESPATH").isEmpty() == false) {
				propertiesFilePath = System.getenv("PROPERTIESPATH"); 
			} else {
				propertiesFilePath = defaultPropertiesFilePath;
			}
			FileInputStream fis = new FileInputStream(propertiesFilePath);
			JSONTokener tokener = new JSONTokener(fis);
			configuration = new JSONObject(tokener);
		} catch (IOException | JSONException ex) {
			System.err.println("Unable to load configuration file, using path " + propertiesFilePath);
			ex.printStackTrace();
		}
	}
	
	public static Object getProperty(String path) {
		return getProperty(path, defaultPropertyName);
	}
	public static Object getProperty(String path, String equipletName) {
		if(configuration == null) initialize();
		JSONObject currentObject = configuration;
		
		String[] pathSegments = path.split("/");
		try {
			for (String pathSegment : pathSegments) {
				currentObject = currentObject.getJSONObject(pathSegment);
			}
			
			if(currentObject.has(equipletName)) {
				return currentObject.get(equipletName);
			} else {
				return currentObject.get(defaultPropertyName);
			}
		} catch (JSONException ex) {
			System.err.println("Unable to retrieve property in configuration, using path " + path);
			ex.printStackTrace();
			return null;
		}
	}	
}
