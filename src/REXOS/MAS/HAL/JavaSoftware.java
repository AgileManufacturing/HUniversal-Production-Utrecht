package HAL;

import libraries.dynamicloader.DynamicClassDescription;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeDBClient;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.knowledgedb_client.Row;

import org.apache.commons.codec.binary.Base64;

import com.google.gson.JsonObject;

public abstract class JavaSoftware {
	private static final String getHalSoftwareForModuleType =
			"SELECT * \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getHalSoftwareForCapabilityName =
			"SELECT * \n" + 
			"FROM JavaSoftware \n" + 
			"WHERE id = ( \n" + 
			"	SELECT halSoftware \n" + 
			"	FROM CapabilityType \n" + 
			"	WHERE name = ? \n" + 
			");"; 
	
	private static final String addHalSoftware =
			"INSERT INTO JavaSoftware \n" + 
			"(buildNumber, className, jarFile) \n" + 
			"VALUES(?, ?, ?);";
	
	public static int deserializeJavaSoftware(KnowledgeDBClient knowledgeDBClient, JsonObject javaSoftware) throws KnowledgeException {
		byte[] jarFile = Base64.decodeBase64(javaSoftware.get("jarFile").getAsString().getBytes());
		return knowledgeDBClient.executeUpdateQuery(addHalSoftware, 
				javaSoftware.get("buildNumber").getAsInt(), javaSoftware.get("className").getAsString(), jarFile);
	}
	
	public static JsonObject serializeJavaSoftwareForModuleIdentifier(KnowledgeDBClient knowledgeDBClient, ModuleIdentifier moduleIdentifier) throws KeyNotFoundException, KnowledgeException {
		JsonObject javaSoftware = new JsonObject();
		Row[] rows = knowledgeDBClient.executeSelectQuery(getHalSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
		javaSoftware.addProperty("buildNumber", (Integer) rows[0].get("buildNumber"));
		javaSoftware.addProperty("className", (String) rows[0].get("className"));
		byte[] jarFile = (byte[]) rows[0].get("jarFile");
		javaSoftware.addProperty("jarFile", new String(Base64.encodeBase64(jarFile)));
		return javaSoftware;
	}

	public static DynamicClassDescription getDynamicClassDescriptionForModuleIdentifier(
			KnowledgeDBClient knowledgeDBClient, ModuleIdentifier moduleIdentifier) {
		try{
			Row[] rows = knowledgeDBClient.executeSelectQuery(getHalSoftwareForModuleType, moduleIdentifier.getManufacturer(), moduleIdentifier.getTypeNumber());
			Integer id = (Integer) rows[0].get("id");
			String className = (String) rows[0].get("className");
			return new DynamicClassDescription(id, className);
		} catch(KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::JavaSoftware::getDynamicClassDescriptionForModuleIdentifier(): Error occured which is considered to be impossible " + ex);
			return null;
		}
	}

	public static DynamicClassDescription getDynamicClassDescriptionForCapabilityName(
			KnowledgeDBClient knowledgeDBClient, String name) {
		try{
			Row[] rows = knowledgeDBClient.executeSelectQuery(getHalSoftwareForCapabilityName, name);
			Integer id = (Integer) rows[0].get("id");
			String className = (String) rows[0].get("className");
			return new DynamicClassDescription(id, className);
		} catch(KnowledgeException | KeyNotFoundException ex) {
			System.err.println("HAL::JavaSoftware::getDynamicClassDescriptionForCapabilityName(): Error occured which is considered to be impossible " + ex);
			return null;
		}
	}

}
