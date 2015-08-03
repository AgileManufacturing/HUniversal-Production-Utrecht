package HAL.dataTypes;

import java.io.Serializable;
import java.util.HashMap;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.libraries.dynamicloader.DynamicClassDescription;
import HAL.libraries.dynamicloader.JarFileLoader;
import HAL.libraries.dynamicloader.JarFileLoaderException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;
/**
 * This class provides methods for serializing and deserializing of JavaSoftware and the loading of jarFiles. 
 * @author Tommas Bakker
 *
 */
public class JavaSoftware implements JarFileLoader, Serializable{
	private static final long serialVersionUID = 4073676842046132307L;
	
	public static final String BUILD_NUMBER = "buildNumber";
	public static final String JAR_FILE = "jarFile";
	public static final String CLASS_NAME = "className";
	
	private static final String getJavaSoftwareForId =
			"SELECT buildNumber, className, jarFile \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ?;"; 
	private static final String getJavaSoftwareForModuleType =
			"SELECT id, buildNumber, className, jarFile \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getJavaSoftwareForCapabilityName =
			"SELECT id, buildNumber, className, jarFile \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM CapabilityType \n" + 
			"	WHERE name = ? \n" + 
			");"; 
	private static final String addJavaSoftware =
			"INSERT INTO JavaSoftware \n" + 
			"(buildNumber, className, jarFile) \n" + 
			"VALUES(?, ?, ?);";
	private static final String updateJavaSoftware =
			"UPDATE JavaSoftware \n" + 
			"SET buildNumber =? AND \n" + 
			"className = ? AND \n" + 
			"jarFile = ? \n;" + 
			"WHERE id = ?;";
	private static final String removeJavaSoftware =
			"DELETE FROM JavaSoftware \n" + 
			"WHERE id = ?;";
	private static final String removeAllUnusedJavaSoftware =
			"DELETE FROM JavaSoftware \n" + 
			"WHERE id NOT IN ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM ModuleType \n" + 
			") AND id NOT IN ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM CapabilityType \n" +
			");";
	
	/**
	 * The map used for the multiton pattern
	 */
	private static HashMap<Integer, JavaSoftware> javaSoftwareInstances = new HashMap<Integer, JavaSoftware>();
	
	public Integer id;
	public int buildNumber;
	public String className;
	public byte[] jarFile;
	
	/**
	 * Constructs a new JavaSoftware object without parameters (and thus does not add it to the multiton).
	 */
	private JavaSoftware() {
		this.id = null;
	}
	/**
	 * Constructs a new JavaSoftware object using the provided KnowledgeDBClient and adds it to the multiton.
	 * @param id
	 * @param buildNumber
	 * @param className
	 * @param knowledgeDBClient
	 */
	private JavaSoftware(int id, int buildNumber, String className, byte[] jarFile) {
		this.id = id;
		this.buildNumber = buildNumber;
		this.className = className;
		this.jarFile = jarFile;
		
		javaSoftwareInstances.put(id, this);
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * If the JavaSoftware has not been instantiated, it will be constructed with a new KnowledgeDBClient.
	 * @param moduleIdentifier
	 * @return
	 */
	public static JavaSoftware getJavaSoftwareForId(int id, KnowledgeDBClient knowledgeDBClient) {
		if(javaSoftwareInstances.containsKey(id) == true) {
			return javaSoftwareInstances.get(id);
		} else {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForId, id);
			int buildNumber = (Integer) rows[0].get("buildNumber");
			String className = (String) rows[0].get("className");
			byte[] jarFile = (byte[]) rows[0].get("jarFile");
			
			return new JavaSoftware(id,buildNumber, className,jarFile);
		}
	}
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static JavaSoftware getJavaSoftwareForModuleIdentifier(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getJavaSoftwareForModuleType, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		
		if(rows.length != 1) {
			return null;
		}
		
		int id = (Integer) rows[0].get("id");
		int buildNumber = (Integer) rows[0].get("buildNumber");
		String className = (String) rows[0].get("className");
		byte[] jarFile = (byte[]) rows[0].get("jarFile");
		
		if(javaSoftwareInstances.containsKey(id) == true) {
			return javaSoftwareInstances.get(id);
		} else {
			return new JavaSoftware(id, buildNumber, className, jarFile);
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
		byte[] jarFile = (byte[]) rows[0].get("jarFile");
		
		if(javaSoftwareInstances.containsKey(id) == true) {
			return javaSoftwareInstances.get(id);
		} else {
			return new JavaSoftware(id, buildNumber, className, jarFile);
		}
	}
	
	
	public static JavaSoftware deSerialize(JSONObject input) throws JSONException {
		JavaSoftware output = new JavaSoftware();
		
		output.buildNumber = input.getInt(BUILD_NUMBER);
		if(input.isNull(JAR_FILE) == false) {
			output.jarFile = Base64.decodeBase64(input.getString(JAR_FILE).getBytes());
		} else {
			output.jarFile = null;
		}
		output.className = input.getString(CLASS_NAME);
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(BUILD_NUMBER, buildNumber);
		if(jarFile != null) {
			output.put(JAR_FILE, new String(Base64.encodeBase64(jarFile)));
		} else {
			output.put(JAR_FILE, JSONObject.NULL);
		}
		output.put(CLASS_NAME, className);
		
		return output;
	}
	
	
	
	/**
	 * Deserializes java software from a JSONObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param javaSoftware
	 * @param knowledgeDBClient
	 * @return the now stored java software
	 * @throws JSONException 
	 */
	public int insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) {
		int id = knowledgeDBClient.executeUpdateQuery(addJavaSoftware, 
				buildNumber, className, jarFile);
		this.id = id;
		javaSoftwareInstances.put(id, this);
		return id;
	}
	/**
	 * This method will update the java software in the knowledge database. 
	 * @param javaSoftware
	 * @throws JSONException 
	 */
	public void updateDatabase(JavaSoftware javaSoftwareToBeUpdated, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(updateJavaSoftware, 
				buildNumber, className, jarFile, javaSoftwareToBeUpdated.id);
		javaSoftwareInstances.put(id, this);
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeJavaSoftware, id);
	}	
	public static void removeUnusedFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeAllUnusedJavaSoftware);
	}
	
	/**
	 * This method will load the jar file from the knowledge database and will return the contents of it.
	 * @throws JarFileLoaderException if the loading of the jarFile failed
	 */
	@Override
	public byte[] loadJarFile() throws JarFileLoaderException {
		return jarFile;
	}
	/**
	 * Constructs a new DynamicClassDescription for this JavaSoftware.
	 * @return
	 */
	public DynamicClassDescription getDynamicClassDescription() {
			return new DynamicClassDescription(id, className, this);
	}
	/**
	 * Returns the build number (used by the dynamic class loader)
	 */
	@Override
	public int getBuildNumber() {
		return buildNumber;
	}
}
