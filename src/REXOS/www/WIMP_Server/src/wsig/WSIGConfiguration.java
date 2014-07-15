/*****************************************************************
JADE - Java Agent DEvelopment Framework is a framework to develop 
multi-agent systems in compliance with the FIPA specifications.
Copyright (C) 2002 TILAB

GNU Lesser General Public License

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation, 
version 2.1 of the License. 

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA  02111-1307, USA.
*****************************************************************/

package wsig;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Iterator;
import java.util.Properties;

import org.apache.log4j.Logger;

import wsig.wsdl.WSDLConstants;

public class WSIGConfiguration extends Properties {

	private static WSIGConfiguration anInstance;
	private static Logger log = Logger.getLogger(WSIGConfiguration.class.getName());

	public static final String WSIG_DEFAULT_CONFIGURATION_FILE = "conf/wsig.properties";
	private static String wsigConfPath;

	// AGENT CONFIGURATION FOR SERVLET
	public static final String KEY_WSIG_AGENT_CLASS_NAME = "wsig.agent";
	public static final String KEY_WSIG_URI = "wsig.uri";
	public static final String KEY_WSIG_CONSOLE_URI = "wsig.console.uri";
	public static final String KEY_WSIG_TIMEOUT = "wsig.timeout";
	
	// DF to WSDL converter
	public final static String KEY_LOCAL_NAMESPACE_PREFIX = "wsdl.localNamespacePrefix";
	public static final String KEY_WSDL_WRITE_ENABLE = "wsdl.writeEnable";
	public static final String KEY_WSDL_DIRECTORY = "wsdl.directory";
	public static final String KEY_WSDL_STYLE = "wsdl.style";
	
	// UDDI repository configuration
	public final static String KEY_UDDI_ENABLE = "uddi.enable";
	public final static String KEY_QUERY_MANAGER_URL = "uddi.queryManagerURL";
	public final static String KEY_LIFE_CYCLE_MANAGER_URL = "uddi.lifeCycleManagerURL";
	public final static String KEY_BUSINESS_KEY = "uddi.businessKey";
	public final static String KEY_USER_NAME = "uddi.userName";
	public final static String KEY_USER_PASSWORD = "uddi.userPassword";
	public final static String KEY_UDDI_TMODEL = "uddi.tmodel.key";

	// UDDI4j configuration
	public final static String KEY_UDDI4J_LOG_ENABLED = "org.uddi4j.logEnabled";
	public final static String KEY_UDDI4J_TRANSPORT_CLASS = "org.uddi4j.TransportClassName";

	// Ontology configuration
	public final static String KEY_ONTO_PREFIX = "onto";
	
	
	/**
	 * Return an instance of the class.
	 * Only one instance is reasonable to use.
	 *
	 * @return a instance
	 */
	public static synchronized WSIGConfiguration getInstance() {
		if (anInstance == null) {
			anInstance = new WSIGConfiguration();
			load();
		}
		return anInstance;
	}

   	public static void init(String _wsigConfPath){
		wsigConfPath = _wsigConfPath;
   	}


	// AGENT CONFIGURATION FOR SERVLET
	public synchronized String getMainHost() {
		return getProperty(jade.core.Profile.MAIN_HOST);
	}
	public synchronized String getMainPort() {
		return getProperty(jade.core.Profile.MAIN_PORT);
	}
	public synchronized String getContainerName() {
		return getProperty(jade.core.Profile.CONTAINER_NAME);
	}
	public synchronized String getLocalPort() {
		return getProperty(jade.core.Profile.LOCAL_PORT);
	}
	public synchronized String getAgentClassName() {
		return getProperty(KEY_WSIG_AGENT_CLASS_NAME);
	}
	public synchronized String getWsigUri() {
		return getProperty(KEY_WSIG_URI);
	}
	public synchronized String getWsigConsoleUri() {
		return getProperty(KEY_WSIG_CONSOLE_URI);
	}
	public synchronized int getWsigTimeout() {
		String timeout = getProperty(KEY_WSIG_TIMEOUT);
		return Integer.parseInt(timeout);
	}
	
   	// DF to WSDL converter
	public synchronized String getLocalNamespacePrefix() {
		return getProperty(KEY_LOCAL_NAMESPACE_PREFIX);
	}

	public synchronized String getWsdlDirectory() {
		return getProperty(KEY_WSDL_DIRECTORY);
	}
	public synchronized void setWsdlDirectory(String wsdlDirectory) {
		setProperty(KEY_WSDL_DIRECTORY, wsdlDirectory);
	}

	public synchronized boolean isWsdlWriteEnable() {
		String wsdlWriteEnable = getProperty(KEY_WSDL_WRITE_ENABLE);
		return "true".equalsIgnoreCase(wsdlWriteEnable);
		
	}

	public synchronized String getWsdlStyle() {
		return getProperty(KEY_WSDL_STYLE);
	}
	
	// UDDI repository configuration
	public synchronized boolean isUddiEnable() {
		String uddiEnable = getProperty(KEY_UDDI_ENABLE);
		return "true".equalsIgnoreCase(uddiEnable);
	}

	public synchronized String getQueryManagerURL() {
		return getProperty(KEY_QUERY_MANAGER_URL);
	}

	public synchronized String getLifeCycleManagerURL() {
		return getProperty(KEY_LIFE_CYCLE_MANAGER_URL);
	}
	
	public synchronized String getBusinessKey() {
		return getProperty(KEY_BUSINESS_KEY);
	}
   	
	public synchronized String getUserName() {
		return getProperty(KEY_USER_NAME);
	}

	public synchronized String getUserPassword() {
		return getProperty(KEY_USER_PASSWORD);
	}
	
	public synchronized String getTModel() {
		return getProperty(KEY_UDDI_TMODEL);
	}
	
	// UDDI4j configuration
	public synchronized String getUDDI4jLogEnabled() {
		return getProperty(KEY_UDDI4J_LOG_ENABLED);
	}

	public synchronized String getUDDI4jTransportClass() {
		return getProperty(KEY_UDDI4J_TRANSPORT_CLASS);
	}

	// Ontology configuration
	public synchronized String getOntoClassname(String ontoName) {
		
		String ontoKey = KEY_ONTO_PREFIX + "." + ontoName;
		String propertyKey;
		Iterator it =keySet().iterator();
		while(it.hasNext()) {
			propertyKey = (String)it.next();
			if (propertyKey.equals(ontoKey))
				return getProperty(propertyKey);
		}
		return null;
	}
	
	/**
	 * adds properties missed.
	 */
	private void setDefaultProperties() {
		setProperty(WSIGConfiguration.KEY_WSIG_URI, "http://localhost:8180/wsig/ws");
		setProperty(WSIGConfiguration.KEY_WSIG_CONSOLE_URI, "http://localhost:8180/wsig");
		setProperty(WSIGConfiguration.KEY_WSIG_TIMEOUT, "30000");
		setProperty(WSIGConfiguration.KEY_WSDL_DIRECTORY, "wsdl");
		setProperty(WSIGConfiguration.KEY_WSDL_WRITE_ENABLE, "false");
		setProperty(WSIGConfiguration.KEY_WSDL_STYLE, WSDLConstants.STYLE_RPC);
		setProperty(WSIGConfiguration.KEY_UDDI_ENABLE, "true");
		setProperty(WSIGConfiguration.KEY_LIFE_CYCLE_MANAGER_URL, "");
		setProperty(WSIGConfiguration.KEY_QUERY_MANAGER_URL, "");
		setProperty(WSIGConfiguration.KEY_USER_NAME, "");
		setProperty(WSIGConfiguration.KEY_USER_PASSWORD, "");
		setProperty(WSIGConfiguration.KEY_BUSINESS_KEY, "");
		setProperty(WSIGConfiguration.KEY_LOCAL_NAMESPACE_PREFIX, "impl");
		setProperty(WSIGConfiguration.KEY_UDDI4J_LOG_ENABLED, "false");
		setProperty(WSIGConfiguration.KEY_UDDI4J_TRANSPORT_CLASS, "org.uddi4j.transport.ApacheAxisTransport");
		setProperty(WSIGConfiguration.KEY_UDDI_TMODEL, "uuid:A035A07C-F362-44dd-8F95-E2B134BF43B4");
	}

	/**
	 * Retrieves configuration.
	 * An internal instance is loaded.
	 */
	public static void load() {
		
		log.info("Loading WSIG configuration file...");
		WSIGConfiguration c = getInstance();
		InputStream is;
		synchronized (c) {
			c.setDefaultProperties();
			
			InputStream input = null;
			if (wsigConfPath != null) {
				try {
					input = new FileInputStream(wsigConfPath);
				} catch (FileNotFoundException e) {
					log.error("WSIG configuration file <<" + wsigConfPath + ">> not found, wsig agent will use default configuration", e);
					return;
				}
			} else {
				input = ClassLoader.getSystemResourceAsStream(WSIG_DEFAULT_CONFIGURATION_FILE);
				if (input == null) {
					log.error("WSIG configuration file <<" + WSIG_DEFAULT_CONFIGURATION_FILE + ">> not found, wsig agent will use default configuration");
					return;
				}
			}
			try {
				is = new BufferedInputStream(input);
				c.load(is);
				is.close();
				log.debug("WSIG configuration file is loaded");

			} catch (IOException e) {
				log.error("WSIG configuration file error reading", e);
			}
		}
	}
}
