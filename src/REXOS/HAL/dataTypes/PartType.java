package HAL.dataTypes;

import java.io.Serializable;

import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

public class PartType implements Serializable {
	private static final long serialVersionUID = 4432359335483148571L;
	
	public static final String TYPE_NUMBER = "typeNumber";
	public static final String PROPERTIES = "properties";
	public static final String GAZEBO_MODEL = "gazeboModel";
	
	/**
	 * SQL query for selecting all the data of partType.
	 * Input: partTypeNumber
	 */
	private static final String getPartType = 
			"SELECT * \n" +
			"FROM PartType \n" +
			"WHERE typeNumber = ?;";
	/**
	 * SQL query for removing all the partType which are obsolete.
	 * Input: -
	 * A partType is considered to be obsolete if there are no parts of that type connected to any equiplet.
	 */
	private static final String getPartTypesWithNoParts = 
			"SELECT typeNumber \n" +
			"FROM PartType \n" +
			"WHERE NOT EXISTS( \n" +
			"	SELECT * \n" +
			"	FROM Part \n" +
			"	WHERE typeNumber = PartType.typeNumber \n" +
			");";
	private static final String getPropertiesForPartTypeNumber = 
			"SELECT partTypeProperties \n" +
			"FROM PartType \n" +
			"WHERE typeNumber = ?;";
	/**
	 * SQL query for adding a partType. Input: partTypeManufacturer,
	 * partTypeTypeNumber, halSoftwareId, rosSoftwareId Ignored if a record
	 * with the same primary key already exists.
	 */
	private static final String addPartType = 
			"INSERT IGNORE INTO PartType \n" +
			"(typeNumber, partTypeProperties, gazeboModel) \n" +
			"VALUES (?, ?, ?);";
	/**
	 * SQL query for removing a part. Input: partManufacturer,
	 * partTypeNumber, partSerialNumber
	 */
	private static final String removePartType = 
			"DELETE FROM PartType \n" +
			"WHERE typeNumber = ?;";
	
	public String partTypeNumber;
	public JSONObject properties;
	public GazeboModel gazeboModel;
	
	public static PartType getSerializedPartTypeByPartTypeNumber(String partTypeNumber, 
			KnowledgeDBClient knowledgeDBClient) throws JSONException {
		PartType output = new PartType();
		output.partTypeNumber = partTypeNumber;
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getPropertiesForPartTypeNumber, 
				partTypeNumber);
		JSONTokener tokener = new JSONTokener((String) rows[0].get("partTypeProperties"));
		output.properties = new JSONObject(tokener);
		
		output.gazeboModel = GazeboModel.getGazeboModelForPartTypeNumber(partTypeNumber, knowledgeDBClient);

		return output;
	}
	
	public static PartType deSerialize(JSONObject input) throws JSONException {
		PartType output = new PartType();
		
		output.partTypeNumber = input.getString(TYPE_NUMBER);
		output.properties = input.getJSONObject(PROPERTIES);
		output.gazeboModel = GazeboModel.deSerialize(input.getJSONObject(GAZEBO_MODEL));
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(PROPERTIES, properties);
		output.put(GAZEBO_MODEL, gazeboModel.serialize());
		
		return output;
	}
	
	/**
	 * This method inserts a partType in the knowledge database.
	 * 
	 * @param partIdentifier
	 * @param type
	 * @return
	 * @throws KnowledgeException
	 * @throws JSONException
	 */
	public void insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) throws JSONException {
		if(existsInDatabase(knowledgeDBClient) == true) {
			updatePartType(knowledgeDBClient);
		} else {
			int gazeboModelId = gazeboModel.insertIntoDatabase(knowledgeDBClient);
			
			knowledgeDBClient.executeUpdateQuery(addPartType, 
					partTypeNumber, properties, gazeboModelId);
		}
	}
	/**
	 * This method updates a partType in the knowledge database. It will
	 * update the software of the partType if the buildNumber of the provided
	 * software is higher than the buildNumber of the currently stored software.
	 * 
	 * @param partIdentifier
	 * @param type
	 * @throws JSONException
	 */
	private void updatePartType(KnowledgeDBClient knowledgeDBClient) {
		if(gazeboModel != null) {	
			GazeboModel currentGazeboModel = GazeboModel.getGazeboModelForPartTypeNumber(partTypeNumber, knowledgeDBClient);
			int currentModelBuildNumber;
			if(currentGazeboModel == null) currentModelBuildNumber = Integer.MIN_VALUE;
			else currentModelBuildNumber = currentGazeboModel.buildNumber;
			
			int newModelBuildNumber = gazeboModel.buildNumber;
			if (newModelBuildNumber > currentModelBuildNumber) {
				// update the gazeboModel
				Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.INFORMATION, "Updating Gazebo model for part " + partTypeNumber);
				gazeboModel.updateGazeboModel(currentGazeboModel, knowledgeDBClient);
			}
		}
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removePartType, partTypeNumber);
		
		gazeboModel.removeFromDatabase(knowledgeDBClient);
	}
	public static void removeUnusedFromDatabase(KnowledgeDBClient knowledgeDBClient) throws JSONException {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getPartTypesWithNoParts);
		for (Row row : rows) {
			String partTypeNumber = (String) row.get("typeNumber");
			PartType part = PartType.getSerializedPartTypeByPartTypeNumber(partTypeNumber, knowledgeDBClient);
			part.removeFromDatabase(knowledgeDBClient);
		}
	}
	/**
	 * This method determines if a partType (identified by the {@link PartIdentifier}) is known in the knowledge database.
	 * 
	 * @param partIdentifier
	 * @return true if the partType is known in the knowledge database, false otherwise.
	 */
	public boolean existsInDatabase(KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getPartType, partTypeNumber);
		if (rows.length == 1) {
			return true;
		} else {
			return false;
		}
	}

}
