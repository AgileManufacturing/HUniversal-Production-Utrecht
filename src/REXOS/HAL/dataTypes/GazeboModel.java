package HAL.dataTypes;

import java.io.Serializable;
import java.util.HashMap;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class GazeboModel implements Serializable {
	private static final long serialVersionUID = -223660362496024821L;
	
	public static final String BUILD_NUMBER = "buildNumber";
	public static final String ZIP_FILE = "zipFile";
	public static final String SDF_FILE_NAME = "sdfFilename";
	public static final String PARENT_LINK = "parentLink";
	public static final String CHILD_LINK = "childLink";
	
	private static final String getGazeboModelForModuleType =
			"SELECT id, buildNumber, sdfFilename, parentLink, childLink, zipFile \n" + 
			"FROM GazeboModel \n" + 
			"WHERE id = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String addGazeboModel =
			"INSERT INTO GazeboModel \n" + 
			"(buildNumber, sdfFilename, parentLink, childLink, zipFile) \n" + 
			"VALUES(?, ?, ?, ?, ?);";
	private static final String updateGazeboModel =
			"UPDATE GazeboModel \n" + 
			"SET buildNumber =? AND \n" + 
			"sdfFilename = ? AND \n" + 
			"parentLink = ? AND \n" + 
			"childLink = ? AND \n" + 
			"zipFile = ? \n;" + 
			"WHERE id = ?;";
	private static final String removeGazeboModel =
			"DELETE FROM GazeboModel \n" + 
			"WHERE id = ?;";
	
	/**
	 * The map used for the multiton pattern
	 */
	private static HashMap<Integer, GazeboModel> gazeboModelInstances = new HashMap<Integer, GazeboModel>();
	
	public Integer id;
	public int buildNumber;
	public byte[] zipFile;
	public String sdfFilename;
	public String parentLink;
	public String childLink;
	
	public GazeboModel() {
		id = null;
	}
	public GazeboModel(int id, int buildNumber, byte[] zipFile, String sdfFilename, String parentLink, String childLink) {
		this.id = id;
		this.buildNumber = buildNumber;
		this.zipFile = zipFile;
		this.sdfFilename = sdfFilename;
		this.parentLink = parentLink;
		this.childLink = childLink;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static GazeboModel getGazeboModelForModuleIdentifier(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboModelForModuleType, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		
		int id = (Integer) rows[0].get("id");
		int buildNumber = (Integer) rows[0].get("buildNumber");
		String sdfFilename = (String) rows[0].get("sdfFilename");
		String parentLink = (String) rows[0].get("parentLink");
		String childLink = (String) rows[0].get("childLink");
		byte[] zipFile = (byte[]) rows[0].get("zipFile");
		
		if(gazeboModelInstances.containsKey(id) == true) {
			return gazeboModelInstances.get(id);
		} else {
			return new GazeboModel(id, buildNumber, zipFile, sdfFilename, parentLink, childLink);
		}
	}
	
	public static GazeboModel deSerialize(JSONObject input) throws JSONException {
		GazeboModel output = new GazeboModel();
		
		output.buildNumber = input.getInt(BUILD_NUMBER);
		output.zipFile = Base64.decodeBase64(input.getString(ZIP_FILE).getBytes());
		output.sdfFilename = input.getString(SDF_FILE_NAME);
		output.parentLink = input.getString(PARENT_LINK);
		output.childLink = input.getString(CHILD_LINK);
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(BUILD_NUMBER, buildNumber);
		output.put(ZIP_FILE, new String(Base64.encodeBase64(zipFile)));
		output.put(SDF_FILE_NAME, sdfFilename);
		output.put(PARENT_LINK, parentLink);
		output.put(CHILD_LINK, childLink);
		
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
		int id = knowledgeDBClient.executeUpdateQuery(addGazeboModel, 
				buildNumber, sdfFilename, parentLink, childLink, zipFile);
		this.id = id;
		gazeboModelInstances.put(id, this);
		return id;
	}
	/**
	 * This method will update the java software in the knowledge database. 
	 * @param javaSoftware
	 * @throws JSONException 
	 */
	public void updateGazeboModel(GazeboModel gazeboModelToBeUpdated, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(updateGazeboModel, 
				buildNumber, sdfFilename, parentLink, childLink, zipFile, gazeboModelToBeUpdated.id);
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeGazeboModel, id);
	}
}
