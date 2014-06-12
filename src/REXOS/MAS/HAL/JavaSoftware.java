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
/**
 * This class provides methods for serializing and deserializing of JavaSoftware and the loading of jarFiles. 
 * @author Tommas Bakker
 *
 */
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
	/**
	 * The map used for the multiton pattern
	 */
	private static HashMap<Integer, JavaSoftware> javaSoftwareInstances = new HashMap<Integer, JavaSoftware>(); 
	/**
	 * Deserializes java software from a JsonObject and stores in in the knowledge database.
	 * This method uses a new KnowledgeDBClient which may cause locking issues. It is recommended to provide your own KnowledgeDBClient.
	 * @param javaSoftware the JsonObject to be used
	 * @return the now stored java software
	 */
	public static JavaSoftware insertJavaSoftware(JsonObject javaSoftware) {
		return insertJavaSoftware(javaSoftware, new KnowledgeDBClient());
	}
	/**
	 * Deserializes java software from a JsonObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param javaSoftware
	 * @param knowledgeDBClient
	 * @return the now stored java software
	 */
	public static JavaSoftware insertJavaSoftware(JsonObject javaSoftware, KnowledgeDBClient knowledgeDBClient) {
		byte[] jarFile = Base64.decodeBase64(javaSoftware.get("jarFile").getAsString().getBytes());
		int buildNumber = getBuildNumber(javaSoftware);
		String className = getClassName(javaSoftware);
		
		int id = knowledgeDBClient.executeUpdateQuery(addJavaSoftware, 
				buildNumber, className, jarFile);
		return new JavaSoftware(id, buildNumber, className, knowledgeDBClient);
	}
	/**
	 * This method allows the extraction of the build number of the javaSoftware from the JsonObject. Used for updating.
	 * @param javaSoftware
	 * @return
	 */
	public static int getBuildNumber(JsonObject javaSoftware) {
		return javaSoftware.get("buildNumber").getAsInt();
	}
	/**
	 * This method allows the extraction of the class name of the javaSoftware from the JsonObject.
	 * @param javaSoftware
	 * @return
	 */
	public static String getClassName(JsonObject javaSoftware) {
		return javaSoftware.get("className").getAsString();
	}
	
	private int id;
	public int getId() {
		return id;
	}
	private int buildNumber;
	@Override
	public int getBuildNumber() {
		return buildNumber;
	}
	private String className;
	public String getClassName() {
		return className;
	}

	private KnowledgeDBClient knowledgeDBClient;
	
	/**
	 * Constructs a new JavaSoftware object using the provided KnowledgeDBClient and adds it to the multiton.
	 * @param id
	 * @param buildNumber
	 * @param className
	 * @param knowledgeDBClient
	 */
	private JavaSoftware(int id, int buildNumber, String className, KnowledgeDBClient knowledgeDBClient) {
		this.id = id;
		this.buildNumber = buildNumber;
		this.className = className;
		this.knowledgeDBClient = knowledgeDBClient;
		
		javaSoftwareInstances.put(id, this);
	}
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * If the JavaSoftware has not been instantiated, it will be constructed with a new KnowledgeDBClient.
	 * @param moduleIdentifier
	 * @return
	 */
	public static JavaSoftware getJavaSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier) {
		try{
			return getJavaSoftwareForModuleIdentifier(moduleIdentifier, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::JavaSoftware::getJavaSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static JavaSoftware getJavaSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
		
		int id = (Integer) rows[0].get("id");
		int buildNumber = (Integer) rows[0].get("buildNumber");
		String className = (String) rows[0].get("className");
		
		if(javaSoftwareInstances.containsKey(id) == true) {
			return javaSoftwareInstances.get(id);
		} else {
			return new JavaSoftware(id, buildNumber, className, knowledgeDBClient);
		}
	}
	/**
	 * This method will get the JavaSoftware associated with the capability. 
	 * If the JavaSoftware has not been instantiated, it will be constructed with a new KnowledgeDBClient.
	 * @param capabilityName
	 * @return
	 */
	public static JavaSoftware getJavaSoftwareForCapabilityName(String capabilityName) {
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
	}
	/**
	 * Constructs a new DynamicClassDescription for this JavaSoftware.
	 * @return
	 */
	public DynamicClassDescription getDynamicClassDescription() {
			return new DynamicClassDescription(id, className, this);
	}
	/**
	 * This method serializes java software from the knowledge database. This method does NOT remove the java software from the knowledge database.
	 * @param knowledgeDBClient
	 * @param moduleIdentifier
	 * @return The JsonObject for the JavaSoftware
	 */
	public JsonObject serialize() {
		JsonObject javaSoftware = new JsonObject();
		Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForId, this.id);
		javaSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
		javaSoftware.addProperty("className", (String) rows[0].get("className"));
		byte[] jarFile = (byte[]) rows[0].get("jarFile");
		javaSoftware.addProperty("jarFile", new String(Base64.encodeBase64(jarFile)));
		return javaSoftware;
	}
	
	/**
	 * This method will load the jar file from the knowledge database and will return the contents of it.
	 * @throws JarFileLoaderException if the loading of the jarFile failed
	 */
	@Override
	public byte[] loadJarFile() throws JarFileLoaderException {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getJarFileForDescription, this.id);
		if(rows.length != 0) {
			byte[] jarFile = (byte[]) rows[0].get("jarFile");
			return jarFile;
		} else {
			throw new JarFileLoaderException("Unable to retrieve the software");
		}
	}
	
	/**
	 * This method will update the java software in the knowledge database. 
	 * @param javaSoftware
	 */
	public void updateJavaSoftware(JsonObject javaSoftware) {
		byte[] jarFile = Base64.decodeBase64(javaSoftware.get("jarFile").getAsString().getBytes());
		int buildNumber = getBuildNumber(javaSoftware);
		String className = getClassName(javaSoftware);
		
		knowledgeDBClient.executeUpdateQuery(updateJavaSoftware, buildNumber, className, jarFile);
	}	
}
