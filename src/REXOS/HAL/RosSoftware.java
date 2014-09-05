package HAL;

import java.util.HashMap;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

import org.apache.commons.codec.binary.Base64;

import com.google.gson.JsonObject;

/**
 * This class provides methods for serializing and deserializing of RosSoftware. 
 * @author Tommas Bakker
 *
 */
public class RosSoftware {
	private static final String getRosSoftwareForId =
			"SELECT id, buildNumber, command, zipFile \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ?;"; 
	private static final String getRosSoftwareForModuleType =
			"SELECT id, buildNumber, command \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT rosSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String addRosSoftware =
			"INSERT INTO RosSoftware \n" + 
			"(buildNumber, command, zipFile) \n" + 
			"VALUES(?, ?, ?);";
	private static final String updateRosSoftware =
			"UPDATE RosSoftware \n" + 
			"SET buildNumber =? AND \n" + 
			"command = ? AND \n" + 
			"zipFile = ?;";
	
	/**
	 * The map used for the multiton pattern
	 */
	private static HashMap<Integer, RosSoftware> rosSoftwareInstances = new HashMap<Integer, RosSoftware>(); 
	/**
	 * Deserializes ros software from a JsonObject and stores in in the knowledge database 
	 * This method uses a new KnowledgeDBClient which may cause locking issues. It is recommended to provide your own KnowledgeDBClient.
	 * @param rosSoftware the JsonObject to be used
	 * @return the now stored ros software
	 */
	public static RosSoftware insertRosSoftware(JsonObject rosSoftware) {
		return insertRosSoftware(rosSoftware, new KnowledgeDBClient());
	}
	/**
	 * Deserializes ros software from a JsonObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param rosSoftware
	 * @param knowledgeDBClient
	 * @return the now stored ros software
	 */
	public static RosSoftware insertRosSoftware(JsonObject rosSoftware, KnowledgeDBClient knowledgeDBClient) {
		byte[] zipFile = Base64.decodeBase64(rosSoftware.get("rosFile").getAsString().getBytes());
		int buildNumber = getBuildNumber(rosSoftware);
		String command = getCommand(rosSoftware);
		
		int id = knowledgeDBClient.executeUpdateQuery(addRosSoftware, 
				buildNumber, command, zipFile);
		return new RosSoftware(id, buildNumber, command, knowledgeDBClient);
	}
	
	/**
	 * This method allows the extraction of the build number of the rosSoftware from the JsonObject. Used for updating.
	 * @param rosSoftware
	 * @return
	 */
	public static int getBuildNumber(JsonObject rosSoftware) {
		return rosSoftware.get("buildNumber").getAsInt();
	}
	/**
	 * This method allows the extraction of the command of the rosSoftware from the JsonObject.
	 * @param rosSoftware
	 * @return
	 */
	public static String getCommand(JsonObject rosSoftware) {
		return rosSoftware.get("command").getAsString();
	}
	
	private int id;
	public int getId() {
		return id;
	}
	private int buildNumber;
	public int getBuildNumber() {
		return buildNumber;
	}
	private String command;
	public String getCommand() {
		return command;
	}

	private KnowledgeDBClient knowledgeDBClient;
	
	/**
	 * Constructs a new RosSoftware object using the provided KnowledgeDBClient and adds it to the multiton.
	 * @param id
	 * @param buildNumber
	 * @param command
	 * @param knowledgeDBClient
	 * @throws KnowledgeException
	 */
	private RosSoftware(int id, int buildNumber, String command, KnowledgeDBClient knowledgeDBClient) throws KnowledgeException {
		this.id = id;
		this.buildNumber = buildNumber;
		this.command = command;
		this.knowledgeDBClient = knowledgeDBClient;
		
		rosSoftwareInstances.put(id, this);
	}
	/**
	 * This method will get the RosSoftware associated with the module. 
	 * If the RosSoftware has not been instantiated, it will be constructed with a new KnowledgeDBClient.
	 * @param moduleIdentifier
	 * @return
	 */
	public static RosSoftware getRosSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier) {
		return getRosSoftwareForModuleIdentifier(moduleIdentifier, new KnowledgeDBClient()); 
	}
	/**
	 * This method will get the RosSoftware associated with the module.
	 * @param moduleIdentifier
	 * @param knowledgeDBClient
	 * @return
	 */
	public static RosSoftware getRosSoftwareForModuleIdentifier(ModuleIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
		
		int id = (Integer) rows[0].get("id");
		int buildNumber = (Integer) rows[0].get("buildNumber");
		String command = (String) rows[0].get("command");
		
		if(rosSoftwareInstances.containsKey(id) == true) {
			return rosSoftwareInstances.get(id);
		} else {
			return new RosSoftware(id, buildNumber, command, knowledgeDBClient);
		}
	}
	
	/**
	 * This method serializes ros software from the knowledge database 
	 * @param knowledgeDBClient
	 * @param moduleIdentifier
	 * @return The JsonObject for the RosSoftware
	 */
	public JsonObject serialize() {
		JsonObject rosSoftware = new JsonObject();
		Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForId, this.id);
		rosSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
		rosSoftware.addProperty("command", (String) rows[0].get("command"));
		byte[] zipFile = (byte[]) rows[0].get("zipFile");
		rosSoftware.addProperty("rosFile", new String(Base64.encodeBase64(zipFile)));
		return rosSoftware;
	}
	
	/**
	 * This method will update the ros software in the knowledge database. 
	 * @param rosSoftware
	 */
	public void updateRosSoftware(JsonObject rosSoftware) {
		byte[] zipFile = Base64.decodeBase64(rosSoftware.get("zipFile").getAsString().getBytes());
		int buildNumber = getBuildNumber(rosSoftware);
		String command = getCommand(rosSoftware);
		
		knowledgeDBClient.executeUpdateQuery(updateRosSoftware, buildNumber, command, zipFile);
	}	
}
