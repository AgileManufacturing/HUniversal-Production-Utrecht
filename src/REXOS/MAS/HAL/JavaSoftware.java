package HAL;

import java.util.HashMap;

import libraries.dynamicloader.DynamicClassDescription;
import libraries.dynamicloader.JarFileLoader;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import org.apache.commons.codec.binary.Base64;

import com.google.gson.JsonObject;

public class JavaSoftware implements JarFileLoader {
	private static final String getJavaSoftwareForId =
			"SELECT id, buildNumber, className, jarFile \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ?;"; 
	private static final String getJavaSoftwareForModuleType =
			"SELECT id, buildNumber, className \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getJavaSoftwareForCapabilityName =
			"SELECT id, buildNumber, className \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM CapabilityType \n" + 
			"	WHERE name = ? \n" + 
			");"; 
	private static final String getJarFileForDescription = 
			"SELECT jarFile \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ?;";
	
	private static final String addJavaSoftware =
			"INSERT INTO JavaSoftware \n" + 
			"(buildNumber, className, jarFile) \n" + 
			"VALUES(?, ?, ?);";
	private static final String updateJavaSoftware =
			"UPDATE JavaSoftware \n" + 
			"SET buildNumber =? AND \n" + 
			"className = ? AND \n" + 
			"jarFile = ?;";
	
	private static HashMap<Integer, JavaSoftware> javaSoftwareInstances = new HashMap<Integer, JavaSoftware>(); 
	/**
	 * Deserializes java software from a JsonObject and stores in in the knowledge database 
	 * @param javaSoftware the JsonObject to be used
	 * @return the now stored java software
	 */
	public static JavaSoftware insertJavaSoftware(JsonObject javaSoftware) {
		try{
			return insertJavaSoftware(javaSoftware, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::JavaSoftware::insertJavaSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static JavaSoftware insertJavaSoftware(JsonObject javaSoftware, KnowledgeDBClient knowledgeDBClient) {
		try {
			byte[] jarFile = Base64.decodeBase64(javaSoftware.get("jarFile").getAsString().getBytes());
			int buildNumber = getBuildNumber(javaSoftware);
			String className = getClassName(javaSoftware);
			
			int id = knowledgeDBClient.executeUpdateQuery(addJavaSoftware, 
					buildNumber, className, jarFile);
			return new JavaSoftware(id, buildNumber, className, knowledgeDBClient);
		} catch (KnowledgeException ex) {
			System.err.println("HAL::JavaSoftware::insertJavaSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	
	public static int getBuildNumber(JsonObject javaSoftware) {
		return javaSoftware.get("buildNumber").getAsInt();
	}
	public static String getClassName(JsonObject javaSoftware) {
		return javaSoftware.get("className").getAsString();
	}
	
	private int id;
	private int buildNumber;
	private String className;
	
	public int getId() {
		return id;
	}
	@Override
	public int getBuildNumber() {
		return buildNumber;
	}
	public String getClassName() {
		return className;
	}

	private KnowledgeDBClient knowledgeDBClient;
	
	private JavaSoftware(int id, int buildNumber, String className, KnowledgeDBClient knowledgeDBClient) throws KnowledgeException {
		this.id = id;
		this.buildNumber = buildNumber;
		this.className = className;
		this.knowledgeDBClient = knowledgeDBClient;
		
		javaSoftwareInstances.put(id, this);
	}
	public static JavaSoftware getJavaSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier) {
		try{
			return getJavaSoftwareForModuleIdentifier(moduleIdentifier, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::JavaSoftware::getJavaSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static JavaSoftware getJavaSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			
			int id = (Integer) rows[0].get("id");
			int buildNumber = (Integer) rows[0].get("buildNumber");
			String className = (String) rows[0].get("className");
			
			if(javaSoftwareInstances.containsKey(id) == true) {
				return javaSoftwareInstances.get(id);
			} else {
				return new JavaSoftware(id, buildNumber, className, knowledgeDBClient);
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::JavaSoftware::getJavaSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static JavaSoftware getJavaSoftwareForCapabilityName(String capabilityName) {
		try{
			KnowledgeDBClient knowledgeDBClient = new KnowledgeDBClient();
			Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForCapabilityName, capabilityName);
			
			int id = (Integer) rows[0].get("id");
			int buildNumber = (Integer) rows[0].get("buildNumber");
			String className = (String) rows[0].get("className");
			
			if(javaSoftwareInstances.containsKey(id) == true) {
				return javaSoftwareInstances.get(id);
			} else {
				return new JavaSoftware(id, buildNumber, className, knowledgeDBClient);
			}
		} catch(KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::JavaSoftware::getJavaSoftwareForCapabilityName(): Error occured which is considered to be impossible " + ex);
			return null;
		}
	}
	
	public DynamicClassDescription getDynamicClassDescription() {
			return new DynamicClassDescription(id, className, this);
	}
	/**
	 * Serializes java software from the knowledge database 
	 * @param knowledgeDBClient
	 * @param moduleIdentifier
	 * @return
	 */
	public JsonObject serialize() {
		try{
			JsonObject javaSoftware = new JsonObject();
			Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForId, this.id);
			javaSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
			javaSoftware.addProperty("className", (String) rows[0].get("className"));
			byte[] jarFile = (byte[]) rows[0].get("jarFile");
			javaSoftware.addProperty("jarFile", new String(Base64.encodeBase64(jarFile)));
			return javaSoftware;
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::JavaSoftware::serializeJavaSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	
	@Override
	public byte[] loadJarFile() throws JarFileLoaderException {
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getJarFileForDescription, this.id);
			if(rows.length != 0) {
				byte[] jarFile = (byte[]) rows[0].get("jarFile");
				return jarFile;
			} else {
				throw new JarFileLoaderException("HAL::Factory::loadJarFile(): Unable to retrieve the software");
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::Factory::loadJarFile(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}

	public void updateJavaSoftware(JsonObject javaSoftware) {
		try {
			byte[] jarFile = Base64.decodeBase64(javaSoftware.get("jarFile").getAsString().getBytes());
			int buildNumber = getBuildNumber(javaSoftware);
			String className = getClassName(javaSoftware);
			
			knowledgeDBClient.executeUpdateQuery(updateJavaSoftware, 
					buildNumber, className, jarFile);
		} catch (KnowledgeException ex) {
			System.err.println("HAL::JavaSoftware::deserializeJavaSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
		}
	}	
}
