package HAL;

import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import org.apache.commons.codec.binary.Base64;

import com.google.gson.JsonObject;

public abstract class RosSoftware {
	private static final String getRosSoftwareForModuleType =
			"SELECT * \n" + 
			"FROM RosSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT rosSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String addRosSoftware =
			"INSERT INTO RosSoftware \n" + 
			"(buildNumber, className, jarFile) \n" + 
			"VALUES(?, ?, ?);";
	
	public static int addRosSoftware(KnowledgeDBClient knowledgeDBClient, JsonObject rosSoftware) throws KnowledgeException {
		byte[] rosFile = Base64.decodeBase64(rosSoftware.get("rosFile").getAsString().getBytes());
		return knowledgeDBClient.executeUpdateQuery(addRosSoftware, 
				rosSoftware.get("buildNumber").getAsInt(), rosSoftware.get("className").getAsString(), rosFile);
	}
	public static JsonObject getRosSoftwareForModuleIdentifier(KnowledgeDBClient knowledgeDBClient, ModuleIdentifier moduleIdentifier) throws KeyNotFoundException, KnowledgeException {
		JsonObject rosSoftware = new JsonObject();
		Row[] rows = knowledgeDBClient.executeSelectQuery(getRosSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
		rosSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
		rosSoftware.addProperty("className", (String) rows[0].get("className"));
		byte[] rosFile = (byte[]) rows[0].get("rosFile");
		rosSoftware.addProperty("rosFile", new String(Base64.encodeBase64(rosFile)));
		return rosSoftware;
	}

}
