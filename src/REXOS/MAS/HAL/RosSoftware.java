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

public class RosSoftware implements JarFileLoader {
	private static final String getRosSoftwareForId =
			"SELECT id, buildNumber, className, jarFile \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ?;"; 
	private static final String getRosSoftwareForModuleType =
			"SELECT id, buildNumber, className \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT rosSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getRosSoftwareForCapabilityName =
			"SELECT id, buildNumber, className \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT rosSoftware \n" + 
			"	FROM CapabilityType \n" + 
			"	WHERE name = ? \n" + 
			");"; 
	private static final String getJarFileForDescription = 
			"SELECT jarFile \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ?;";
	
	private static final String addRosSoftware =
			"INSERT INTO RosSoftware \n" + 
			"(buildNumber, className, jarFile) \n" + 
			"VALUES(?, ?, ?);";
	private static final String updateRosSoftware =
			"UPDATE RosSoftware \n" + 
			"SET buildNumber =? AND \n" + 
			"className = ? AND \n" + 
			"jarFile = ?;";
	
	private static HashMap<Integer, RosSoftware> rosSoftwareInstances = new HashMap<Integer, RosSoftware>(); 
	/**
	 * Deserializes ros software from a JsonObject and stores in in the knowledge database 
	 * @param javaSoftware the JsonObject to be used
	 * @return the now stored java software
	 */
	public static RosSoftware insertRosSoftware(JsonObject rosSoftware) {
		try{
			return insertRosSoftware(rosSoftware, new KnowledgeDBClient());
		} catch (KnowledgeException ex) {
			System.err.println("HAL::RosSoftware::deserializeRosSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static RosSoftware insertRosSoftware(JsonObject rosSoftware, KnowledgeDBClient knowledgeDBClient) {
		try {
			byte[] jarFile = Base64.decodeBase64(rosSoftware.get("rosFile").getAsString().getBytes());
			int buildNumber = getBuildNumber(rosSoftware);
			String className = getClassName(rosSoftware);
			
			int id = knowledgeDBClient.executeUpdateQuery(addRosSoftware, 
					buildNumber, className, jarFile);
			return new RosSoftware(id, buildNumber, className, knowledgeDBClient);
		} catch (KnowledgeException ex) {
			System.err.println("HAL::RosSoftware::deserializeRosSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	
	public static int getBuildNumber(JsonObject rosSoftware) {
		return rosSoftware.get("buildNumber").getAsInt();
	}
	public static String getClassName(JsonObject rosSoftware) {
		return rosSoftware.get("className").getAsString();
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
	
	private RosSoftware(int id, int buildNumber, String className, KnowledgeDBClient knowledgeDBClient) throws KnowledgeException {
		this.id = id;
		this.buildNumber = buildNumber;
		this.className = className;
		this.knowledgeDBClient = knowledgeDBClient;
		
		rosSoftwareInstances.put(id, this);
	}
	public static RosSoftware getRosSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier) {
		try {
			return getRosSoftwareForModuleIdentifier(moduleIdentifier, new KnowledgeDBClient()); 
		} catch (KnowledgeException ex) {
			System.err.println("HAL::RosSoftware::getRosSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static RosSoftware getRosSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			
			int id = (Integer) rows[0].get("id");
			int buildNumber = (Integer) rows[0].get("buildNumber");
			String className = (String) rows[0].get("className");
			
			if(rosSoftwareInstances.containsKey(id) == true) {
				return rosSoftwareInstances.get(id);
			} else {
				return new RosSoftware(id, buildNumber, className, knowledgeDBClient);
			}
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::RosSoftware::getRosSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	public static RosSoftware getRosSoftwareForCapabilityName(String capabilityName) {
		try{
			KnowledgeDBClient knowledgeDBClient = new KnowledgeDBClient();
			Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForCapabilityName, capabilityName);
			
			int id = (Integer) rows[0].get("id");
			int buildNumber = (Integer) rows[0].get("buildNumber");
			String className = (String) rows[0].get("className");
			
			if(rosSoftwareInstances.containsKey(id) == true) {
				return rosSoftwareInstances.get(id);
			} else {
				return new RosSoftware(id, buildNumber, className, knowledgeDBClient);
			}
		} catch(KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::RosSoftware::getRosSoftwareForCapabilityName(): Error occured which is considered to be impossible " + ex);
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
			Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForId, this.id);
			javaSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
			javaSoftware.addProperty("className", (String) rows[0].get("className"));
			byte[] jarFile = (byte[]) rows[0].get("jarFile");
			javaSoftware.addProperty("jarFile", new String(Base64.encodeBase64(jarFile)));
			return javaSoftware;
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::RosSoftware::serializeRosSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	
	@Override
	public byte[] loadJarFile() throws JarFileLoaderException {
		try {
			Row[] rows = knowledgeDBClient.executeSelectQuery(getJarFileForDescription, this.id);
			if(rows.length == 0) {
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

	public void updateRosSoftware(JsonObject javaSoftware) {
		try {
			byte[] jarFile = Base64.decodeBase64(javaSoftware.get("jarFile").getAsString().getBytes());
			int buildNumber = getBuildNumber(javaSoftware);
			String className = getClassName(javaSoftware);
			
			knowledgeDBClient.executeUpdateQuery(updateRosSoftware, 
					buildNumber, className, jarFile);
		} catch (KnowledgeException ex) {
			System.err.println("HAL::RosSoftware::deserializeRosSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
		}
	}	
}
