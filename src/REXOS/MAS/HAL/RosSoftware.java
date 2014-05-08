package HAL;

import java.util.HashMap;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import org.apache.commons.codec.binary.Base64;

import com.google.gson.JsonObject;

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
	private static final String getRosSoftwareForCapabilityName =
			"SELECT id, buildNumber, command \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT rosSoftware \n" + 
			"	FROM CapabilityType \n" + 
			"	WHERE name = ? \n" + 
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
			System.out.println(rosSoftware.toString());
			byte[] zipFile = Base64.decodeBase64(rosSoftware.get("rosFile").getAsString().getBytes());
			int buildNumber = getBuildNumber(rosSoftware);
			String command = getCommand(rosSoftware);
			
			int id = knowledgeDBClient.executeUpdateQuery(addRosSoftware, 
					buildNumber, command, zipFile);
			return new RosSoftware(id, buildNumber, command, knowledgeDBClient);
		} catch (KnowledgeException ex) {
			System.err.println("HAL::RosSoftware::deserializeRosSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	
	public static int getBuildNumber(JsonObject rosSoftware) {
		return rosSoftware.get("buildNumber").getAsInt();
	}
	public static String getCommand(JsonObject rosSoftware) {
		return rosSoftware.get("command").getAsString();
	}
	
	private int id;
	private int buildNumber;
	private String command;
	
	public int getId() {
		return id;
	}
	public int getBuildNumber() {
		return buildNumber;
	}
	public String getCommandName() {
		return command;
	}

	private KnowledgeDBClient knowledgeDBClient;
	
	private RosSoftware(int id, int buildNumber, String command, KnowledgeDBClient knowledgeDBClient) throws KnowledgeException {
		this.id = id;
		this.buildNumber = buildNumber;
		this.command = command;
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
			String command = (String) rows[0].get("command");
			
			if(rosSoftwareInstances.containsKey(id) == true) {
				return rosSoftwareInstances.get(id);
			} else {
				return new RosSoftware(id, buildNumber, command, knowledgeDBClient);
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
			String command = (String) rows[0].get("command");
			
			if(rosSoftwareInstances.containsKey(id) == true) {
				return rosSoftwareInstances.get(id);
			} else {
				return new RosSoftware(id, buildNumber, command, knowledgeDBClient);
			}
		} catch(KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::RosSoftware::getRosSoftwareForCapabilityName(): Error occured which is considered to be impossible " + ex);
			return null;
		}
	}
	
	/**
	 * Serializes java software from the knowledge database 
	 * @param knowledgeDBClient
	 * @param moduleIdentifier
	 * @return
	 */
	public JsonObject serialize() {
		try{
			JsonObject rosSoftware = new JsonObject();
			Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForId, this.id);
			rosSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
			rosSoftware.addProperty("command", (String) rows[0].get("command"));
			byte[] zipFile = (byte[]) rows[0].get("zipFile");
			rosSoftware.addProperty("rosFile", new String(Base64.encodeBase64(zipFile)));
			return rosSoftware;
		} catch (KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::RosSoftware::serializeRosSoftwareForModuleIdentifier(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
			return null;
		}
	}
	
	public void updateRosSoftware(JsonObject rosSoftware) {
		try {
			byte[] zipFile = Base64.decodeBase64(rosSoftware.get("zipFile").getAsString().getBytes());
			int buildNumber = getBuildNumber(rosSoftware);
			String command = getCommand(rosSoftware);
			
			knowledgeDBClient.executeUpdateQuery(updateRosSoftware, 
					buildNumber, command, zipFile);
		} catch (KnowledgeException ex) {
			System.err.println("HAL::RosSoftware::deserializeRosSoftware(): Error occured which is considered to be impossible" + ex);
			ex.printStackTrace();
		}
	}	
}
