package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class GazeboLink implements Serializable {
	private static final long serialVersionUID = -5579471522584621980L;
	
	public static final String LINK_NAME = "linkName";
	public static final String MAX_ACCELERATION = "maxAcceleration";
	
	private static final String getGazeboLinksForModuleType =
			"SELECT gazeboModel, linkName, maxAcceleration \n" + 
			"FROM GazeboLink \n" + 
			"WHERE gazeboModel = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getGazeboLinksForGazeboModel =
			"SELECT gazeboModel, linkName, maxAcceleration \n" + 
			"FROM GazeboLink \n" + 
			"WHERE gazeboModel = ?;"; 
	private static final String getGazeboLinksForPartType =
			"SELECT gazeboModel, linkName, maxAcceleration \n" + 
			"FROM GazeboLink \n" + 
			"WHERE gazeboModel = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM PartType \n" + 
			"	WHERE typeNumber = ? \n" + 
			");";
	private static final String addGazeboLink =
			"INSERT INTO GazeboLink \n" + 
			"(gazeboModel, linkName, maxAcceleration) \n" + 
			"VALUES(?, ?, ?);";
	private static final String updateGazeboLink =
			"UPDATE GazeboLink \n" + 
			"SET maxAcceleration = ? \n" +
			"WHERE gazeboModel = ? AND \n" + 
			"	linkName = ?";
	private static final String removeGazeboLink =
			"DELETE FROM GazeboLink \n" + 
			"WHERE gazeboModel = ? AND \n" + 
			"	linkName = ?";
	
	public GazeboModel gazeboModel;
	public String linkName;
	public double maxAcceleration;
	
	public GazeboLink() {
	}
	public GazeboLink(GazeboModel gazeboModel, String linkName, double maxAcceleration) {
		this.gazeboModel = gazeboModel;
		this.linkName = linkName;
		this.maxAcceleration = maxAcceleration;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboLink> getGazeboLinksForModuleIdentifier(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboLinksForModuleType, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		
		ArrayList<GazeboLink> links = new ArrayList<GazeboLink>();
		for (Row row : rows) {
			String linkName = (String) row.get("linkName");
			double maxAcceleration = (double) row.get("maxAcceleration");
			
			GazeboModel gazeboModel = GazeboModel.getGazeboModelForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
			links.add(new GazeboLink(gazeboModel, linkName, maxAcceleration));
		}
		return links;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboLink> getGazeboLinksForGazeboModel(GazeboModel model, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboLinksForGazeboModel, model.id);
		
		ArrayList<GazeboLink> links = new ArrayList<GazeboLink>();
		for (Row row : rows) {
			String linkName = (String) row.get("linkName");
			double maxAcceleration = (double) row.get("maxAcceleration");
			
			links.add(new GazeboLink(model, linkName, maxAcceleration));
		}
		return links;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the part. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboLink> getGazeboLinksForPartTypeNumber(String partTypeNumber, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboLinksForPartType, partTypeNumber);
		
		ArrayList<GazeboLink> links = new ArrayList<GazeboLink>();
		for (Row row : rows) {
			String linkName = (String) row.get("linkName");
			double maxAcceleration = (double) row.get("maxAcceleration");
			
			GazeboModel gazeboModel = GazeboModel.getGazeboModelForPartTypeNumber(partTypeNumber, knowledgeDBClient);
			links.add(new GazeboLink(gazeboModel, linkName, maxAcceleration));
		}
		return links;
	}
	public static GazeboLink deSerialize(JSONObject input, GazeboModel gazeboModel) throws JSONException {
		GazeboLink output = new GazeboLink();
		
		output.gazeboModel = gazeboModel;
		output.linkName = input.getString(LINK_NAME);
		output.maxAcceleration = input.getDouble(MAX_ACCELERATION);
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(LINK_NAME, linkName);
		output.put(MAX_ACCELERATION, maxAcceleration);
		
		return output;
	}
	
	/**
	 * Deserializes java software from a JSONObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param javaSoftware
	 * @param knowledgeDBClient
	 * @return the now stored java software
	 * @throws JSONException 
	 */
	public void insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(addGazeboLink, 
				gazeboModel.id, linkName, maxAcceleration);
	}
	/**
	 * This method will update the java software in the knowledge database. 
	 * @param javaSoftware
	 * @throws JSONException 
	 */
	public void updateGazeboModel(GazeboLink gazeboLinkToBeUpdated, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(updateGazeboLink, maxAcceleration, 
				gazeboLinkToBeUpdated.gazeboModel.id, gazeboLinkToBeUpdated.linkName);
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeGazeboLink, gazeboModel.id, linkName);
	}
}
