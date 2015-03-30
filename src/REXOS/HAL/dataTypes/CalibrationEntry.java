package HAL.dataTypes;

import java.io.Serializable;
import java.text.DateFormat;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class CalibrationEntry implements Serializable {
	private static final long serialVersionUID = 8500472034490784779L;
	
	public static final String DATE = "date";
	public static final String DATA = "data";
	public static final String MODULE_SET = "moduleSet";
	
	/**
	 * SQL query for selecting all the associated ModuleCalibrationData for a module (which is identified using a {@link ModuleIdentifier}).
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 * ModuleCalibrationData is associated when at least one of the ModuleIdentifiers in the module set matches the ModuleIdentifier of this module.
	 */
	private static final String getAllModuleCalibrationDataForModule = 
			"SELECT id, date, properties \n" + 
			"FROM ModuleCalibration \n" + 
			"WHERE id IN ( \n" +
			"	SELECT ModuleCalibration \n" + 
			"	FROM ModuleCalibrationModuleSet \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? AND \n" + 
			"		serialNumber = ? \n" +
			"); \n";
	/**
	 * SQL query for selecting all the {@link ModuleIdentifier} in the moduleSet of the ModuleCalibrationData.
	 * Input: ModuleCalibrationId
	 */
	private static final String getModuleSetForModuleCalibrationData = 
			"SELECT manufacturer, typeNumber, serialNumber \n" + 
			"FROM ModuleCalibrationModuleSet \n" +
			"WHERE moduleCalibration = ?;";
	/**
	 * SQL query for adding ModuleCalibrationData.
	 * Input: date, properties
	 */
	private static final String addModuleCalibrationData = 
			"INSERT INTO ModuleCalibration \n" + 
			"(date, properties) \n" + 
			"VALUES(?, ?);";
	/**
	 * SQL query for adding modules to ModuleCalibrationModuleSet.
	 * Input: ModuleCalibrationDataId, moduleManufacturer, moduleTypeNumber, moduleSerialNumber
	 */
	private static final String addModuleToModuleCalibrationData = 
			"INSERT INTO ModuleCalibrationModuleSet \n" + 
			"(moduleCalibration, manufacturer, typeNumber, serialNumber) \n" +
			"VALUES(?, ?, ?, ?);";
	/**
	 * SQL query for removing a ModuleCalibrationModuleSet.
	 * Input: ModuleCalibrationDataId
	 */
	private static final String removeModuleSetOfModuleCalibrationData = 
			"DELETE FROM ModuleCalibrationModuleSet \n" + 
			"WHERE moduleCalibration = ?;";
	/**
	 * SQL query for removing a ModuleCalibrationEntry.
	 * Input: ModuleCalibrationDataId
	 */
	private static final String removeModuleCalibrationData = 
			"DELETE FROM ModuleCalibration \n" + 
			"WHERE id = ?;";
	
	/**
	 * The map used for the multiton pattern
	 */
	private static HashMap<Integer, CalibrationEntry> calibrationEntryInstances = new HashMap<Integer, CalibrationEntry>();
	
	private static DateFormat format = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
	public Integer id;
	public Date date;
	public JSONObject data;
	public ArrayList<ModuleIdentifier> moduleSet;
	
	public CalibrationEntry() {
		id = null;
		moduleSet = new ArrayList<ModuleIdentifier>();
	}
	
	public static ArrayList<CalibrationEntry> getCalibrationDataForModule(ModuleIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) throws JSONException, ParseException {
		ArrayList<CalibrationEntry> output = new ArrayList<CalibrationEntry>();
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getAllModuleCalibrationDataForModule, 
				moduleIdentifier.manufacturer, moduleIdentifier.typeNumber, moduleIdentifier.serialNumber);
		for (Row row : rows) {
			int id = (int) row.get("id");
			if(calibrationEntryInstances.containsKey(id) == true) {
				output.add(calibrationEntryInstances.get(id));
			} else {
				CalibrationEntry entry = new CalibrationEntry();
				entry.id = (int) row.get("id");
				JSONTokener tokener = new JSONTokener((String) row.get("properties"));
				entry.data = new JSONObject(tokener);
				entry.date = format.parse((String) row.get("date"));
				
				Row[] moduleSetRows = knowledgeDBClient.executeSelectQuery(getModuleSetForModuleCalibrationData, row.get("id"));
				for (Row moduleSetRow : moduleSetRows) {
					entry.moduleSet.add(new ModuleIdentifier((String) moduleSetRow.get("manufacturer"), 
							(String) moduleSetRow.get("typeNumber"), (String) moduleSetRow.get("serialNumber")));
				}
				
				output.add(entry);
			}
		}
		return output;
	}
	
	public static CalibrationEntry deSerialize(JSONObject input) throws JSONException, ParseException {
		CalibrationEntry output = new CalibrationEntry();
		
		output.date = format.parse(input.getString(DATE));
		output.data = input.getJSONObject(DATA);
		
		JSONArray moduleSet = input.getJSONArray(MODULE_SET);
		for (int i = 0; i < moduleSet.length(); i++) {
			output.moduleSet.add(ModuleIdentifier.deSerialize(moduleSet.getJSONObject(i)));
		}
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(DATE, format.format(date));
		output.put(DATA, data);
		
		JSONArray moduleSet = new JSONArray();
		for (int i = 0; i < this.moduleSet.size(); i++) {
			moduleSet.put(this.moduleSet.get(i).serialize());
		}
		output.put(MODULE_SET, moduleSet);
		
		return output;
	}
	
	/**
	 * This method will deserialize all the moduleCalibration data provided and
	 * store it in the knowledge database.
	 * 
	 * @param moduleCalibrationEntries
	 * @throws JSONException
	 */
	public int insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) {
		int calibrationDataId = knowledgeDBClient.executeUpdateQuery(addModuleCalibrationData, format.format(date), data.toString());
		for (ModuleIdentifier moduleSetEntry : moduleSet) {
			knowledgeDBClient.executeUpdateQuery(addModuleToModuleCalibrationData, 
					calibrationDataId, moduleSetEntry.manufacturer, moduleSetEntry.typeNumber, moduleSetEntry.serialNumber);
		}
		id = calibrationDataId;
		return calibrationDataId;
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeModuleSetOfModuleCalibrationData, id);
		knowledgeDBClient.executeUpdateQuery(removeModuleCalibrationData, id);
	}
}
