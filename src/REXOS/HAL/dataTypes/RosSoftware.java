package HAL.dataTypes;

import java.io.Serializable;
import java.util.HashMap;

import org.apache.commons.codec.binary.Base64;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

import org.json.JSONException;
import org.json.JSONObject;

/**
 * This class provides methods for serializing and deserializing of RosSoftware. 
 * @author Tommas Bakker
 *
 */
public class RosSoftware implements Serializable{
	private static final long serialVersionUID = 4218830385800734123L;
	
	public static final String BUILD_NUMBER = "buildNumber";
	public static final String ROS_FILE = "rosFile";
	public static final String COMMAND = "command";
	
	private static final String getRosSoftwareForId =
			"SELECT buildNumber, command, zipFile \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ?;"; 
	private static final String getRosSoftwareForModuleType =
			"SELECT id, buildNumber, command, zipFile \n" + 
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
			"zipFile = ?; \n" + 
			"WHERE id = ?;";
	private static final String removeRosSoftware =
			"DELETE FROM RosSoftware \n" + 
			"WHERE id = ?;";
	private static final String removeAllUnusedRosSoftware =
			"DELETE FROM RosSoftware \n" + 
			"WHERE id NOT IN ( \n" + 
			"	SELECT rosSoftware \n" + 
			"	FROM ModuleType \n" + 
			");";
	
	/**
	 * The map used for the multiton pattern
	 */
	private static HashMap<Integer, RosSoftware> rosSoftwareInstances = new HashMap<Integer, RosSoftware>();
	
	public Integer id;
	public int buildNumber;
	public String command;
	public byte[] rosFile;
	
	/**
	 * Constructs a new RosSoftware object without parameters (and thus does not add it to the multiton).
	 */
	private RosSoftware() {
		this.id = null;
	}
	/**
	 * Constructs a new RosSoftware object using the provided KnowledgeDBClient and adds it to the multiton.
	 * @param id
	 * @param buildNumber
	 * @param command
	 * @param knowledgeDBClient
	 * @throws KnowledgeException
	 */
	private RosSoftware(int id, int buildNumber, String command, byte[] rosFile, KnowledgeDBClient knowledgeDBClient) throws KnowledgeException {
		this.id = id;
		this.buildNumber = buildNumber;
		this.command = command;
		this.rosFile = rosFile;
		
		rosSoftwareInstances.put(id, this);
	}
	
	
	
	/**
	 * This method will get the RosSoftware associated with the module.
	 * @param moduleIdentifier
	 * @param knowledgeDBClient
	 * @return
	 */
	public static RosSoftware getRosSoftwareForId(int id, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForId, id);
		
		int buildNumber = (Integer) rows[0].get("buildNumber");
		String command = (String) rows[0].get("command");
		byte[] rosFile = (byte[]) rows[0].get("zipFile");
		
		if(rosSoftwareInstances.containsKey(id) == true) {
			return rosSoftwareInstances.get(id);
		} else {
			return new RosSoftware(id, buildNumber, command, rosFile, knowledgeDBClient);
		}
	}
	
	/**
	 * This method will get the RosSoftware associated with the module.
	 * @param moduleIdentifier
	 * @param knowledgeDBClient
	 * @return
	 */
	public static RosSoftware getRosSoftwareForModuleIdentifier(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForModuleType, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		
		if(rows.length != 1) {
			return null;
		}
		
		int id = (Integer) rows[0].get("id");
		int buildNumber = (Integer) rows[0].get("buildNumber");
		String command = (String) rows[0].get("command");
		byte[] rosFile = (byte[]) rows[0].get("zipFile");
		
		if(rosSoftwareInstances.containsKey(id) == true) {
			return rosSoftwareInstances.get(id);
		} else {
			return new RosSoftware(id, buildNumber, command, rosFile, knowledgeDBClient);
		}
	}
	
	public static RosSoftware deSerialize(JSONObject input) throws JSONException {
		RosSoftware output = new RosSoftware();
		
		output.buildNumber = input.getInt(BUILD_NUMBER);
		if(input.isNull(ROS_FILE) == false) {
			output.rosFile = Base64.decodeBase64(input.getString(ROS_FILE).getBytes());
		} else {
			output.rosFile = null;
		}
		output.command = input.getString(COMMAND);
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(BUILD_NUMBER, buildNumber);
		if(rosFile != null) {
			output.put(ROS_FILE, new String(Base64.encodeBase64(rosFile)));
		} else {
			output.put(ROS_FILE, JSONObject.NULL);
		}
		output.put(COMMAND, command);
		
		return output;
	}
	
	
	/**
	 * Deserializes ros software from a JSONObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param rosSoftware
	 * @param knowledgeDBClient
	 * @return the now stored ros software
	 * @throws JSONException 
	 */
	public int insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) {
		int id = knowledgeDBClient.executeUpdateQuery(addRosSoftware, 
				buildNumber, command, rosFile);
		this.id = id;
		rosSoftwareInstances.put(id, this);
		return id;
	}
	/**
	 * This method will update the ros software in the knowledge database. 
	 * @param rosSoftware
	 * @throws JSONException 
	 */
	public void updateDatabase(RosSoftware rosSoftware, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(updateRosSoftware, buildNumber, command, rosFile, rosSoftware.id);
		rosSoftwareInstances.put(id, this);
	}	
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeRosSoftware, id);
	}	
	public static void removeUnusedFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeAllUnusedRosSoftware);
	}
	
}
