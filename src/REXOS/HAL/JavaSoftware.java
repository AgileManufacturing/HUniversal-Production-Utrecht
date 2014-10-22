package HAL;

import java.util.HashMap;

import org.apache.commons.codec.binary.Base64;

import HAL.libraries.dynamicloader.DynamicClassDescription;
import HAL.libraries.dynamicloader.JarFileLoader;
import HAL.libraries.dynamicloader.JarFileLoaderException;
import HAL.libraries.knowledgedb_client.KeyNotFoundException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

import org.json.JSONException;
import org.json.JSONObject;
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
	 * Deserializes java software from a JSONObject and stores in in the knowledge database.
	 * This method uses a new KnowledgeDBClient which may cause locking issues. It is recommended to provide your own KnowledgeDBClient.
	 * @param javaSoftware the JSONObject to be used
	 * @return the now stored java software
	 * @throws JSONException 
	 */
	public static JavaSoftware insertJavaSoftware(JSONObject javaSoftware) throws JSONException {
		return insertJavaSoftware(javaSoftware, new KnowledgeDBClient());
	}
	/**
	 * Deserializes java software from a JSONObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param javaSoftware
	 * @param knowledgeDBClient
	 * @return the now stored java software
	 * @throws JSONException 
	 */
	public static JavaSoftware insertJavaSoftware(JSONObject javaSoftware, KnowledgeDBClient knowledgeDBClient) throws JSONException {
		byte[] jarFile = Base64.decodeBase64(javaSoftware.getString("jarFile").getBytes());
		int buildNumber = getBuildNumber(javaSoftware);
		String className = getClassName(javaSoftware);
		
		int id = knowledgeDBClient.executeUpdateQuery(addJavaSoftware, 
				buildNumber, className, jarFile);
		return new JavaSoftware(id, buildNumber, className, knowledgeDBClient);
	}
	/**
	 * This method allows the extraction of the build number of the javaSoftware from the JSONObject. Used for updating.
	 * @param javaSoftware
	 * @return
	 * @throws JSONException 
	 */
	public static int getBuildNumber(JSONObject javaSoftware) throws JSONException {
		return javaSoftware.getInt("buildNumber");
	}
	/**
	 * This method allows the extraction of the class name of the javaSoftware from the JSONObject.
	 * @param javaSoftware
	 * @return
	 * @throws JSONException 
	 */
	public static String getClassName(JSONObject javaSoftware) throws JSONException {
		return javaSoftware.getString("className");
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

	public static <T> JavaSoftware getJavaSoftwareForIdentifier(T aapkip) {
		if(aapkip.getClass().equals(String.class)) {
			return getJavaSoftwareForCapabilityIdentifier((String)aapkip);
		}
		else { 
			return getJavaSoftwareForModuleIdentifier((ModuleIdentifier)aapkip);
		}
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
	public static JavaSoftware getJavaSoftwareForCapabilityIdentifier(String capabilityName) {
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
	 * @return The JSONObject for the JavaSoftware
	 * @throws JSONException 
	 */
	public JSONObject serialize() throws JSONException {
		JSONObject javaSoftware = new JSONObject();
		Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForId, this.id);
		javaSoftware.put("buildNumber", (Integer) rows[0].get("buildNumber"));
		javaSoftware.put("className", (String) rows[0].get("className"));
		byte[] jarFile = (byte[]) rows[0].get("jarFile");
		javaSoftware.put("jarFile", new String(Base64.encodeBase64(jarFile)));
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
	 * @throws JSONException 
	 */
	public void updateJavaSoftware(JSONObject javaSoftware) throws JSONException {
		byte[] jarFile = Base64.decodeBase64(javaSoftware.getString("jarFile").getBytes());
		int buildNumber = getBuildNumber(javaSoftware);
		String className = getClassName(javaSoftware);
		
		knowledgeDBClient.executeUpdateQuery(updateJavaSoftware, buildNumber, className, jarFile);
	}	
}
