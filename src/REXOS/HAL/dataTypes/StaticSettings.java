package HAL.dataTypes;

import java.io.Serializable;
import java.text.ParseException;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class StaticSettings implements Serializable{
	private static final long serialVersionUID = -3990335286891678647L;
	
	public static final String MODULE_IDENTIFIER = "moduleIdentifier";
	public static final String MODULE_CONFIGURATION_PROPERTIES = "properties";
	public static final String MODULE_TYPE = "type";
	public static final String MODULE_CALIBRATION_DATA = "calibrationData";

	/**
	 * SQL query for adding a module which is connected to the mountPlate.
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber, moduleProperties, equiplet, mountPointX, mountPointY
	 * The module is added to the right of the nested set tree.
	 */
	private static final String addModule = 
			"INSERT INTO Module \n" +
			"(manufacturer, typeNumber, serialNumber, moduleProperties) \n" +
			"VALUES (?, ?, ?, ?);";
	/**
	 * SQL query for selecting all the data of module.
	 * Input: moduleManufacturer, moduleTypeNumber, moduleSerialNumber 
	 */
	private static final String getModuleForModuleIdentifier = 
			"SELECT * \n" +
			"FROM Module \n" +
			"WHERE manufacturer = ? AND \n" +
			"	typeNumber = ? AND \n" + 
			"	serialNumber = ?;";
	
	public ModuleIdentifier moduleIdentifier;
	public JSONObject moduleConfigurationProperties;
	public ModuleType moduleType;
	public ArrayList<CalibrationEntry> calibrationData;
	
	public StaticSettings() {
		calibrationData = new ArrayList<CalibrationEntry>();
		moduleType = new ModuleType();
	}
	
	public static StaticSettings getStaticSettingsForModuleIdentifier(ModuleIdentifier moduleIdentifier, 
			KnowledgeDBClient knowledgeDBClient) throws JSONException, ParseException {
		StaticSettings output = new StaticSettings();
		output.moduleIdentifier = moduleIdentifier;
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleForModuleIdentifier, 
				moduleIdentifier.manufacturer, moduleIdentifier.typeNumber, moduleIdentifier.serialNumber);
		JSONTokener tokener = new JSONTokener((String) rows[0].get("moduleProperties"));
		output.moduleConfigurationProperties = new JSONObject(tokener);
		
		output.moduleType = ModuleType.getSerializedModuleTypeByModuleTypeIdentifier(moduleIdentifier, knowledgeDBClient);
		
		output.calibrationData = CalibrationEntry.getCalibrationDataForModule(moduleIdentifier, knowledgeDBClient);
		
		return output;
	}
	
	public static StaticSettings deSerialize(JSONObject input) throws JSONException, ParseException {
		StaticSettings output = new StaticSettings();
		
		output.moduleIdentifier = ModuleIdentifier.deSerialize(input.getJSONObject(MODULE_IDENTIFIER));
		output.moduleConfigurationProperties = input.getJSONObject(MODULE_CONFIGURATION_PROPERTIES);
		
		output.moduleType = ModuleType.deSerialize(input.getJSONObject(MODULE_TYPE), output.moduleIdentifier);
		
		JSONArray calibrationData = input.getJSONArray(MODULE_CALIBRATION_DATA);
		for (int i = 0; i < calibrationData.length(); i++) {
			output.calibrationData.add(CalibrationEntry.deSerialize(calibrationData.getJSONObject(i)));
		}
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(MODULE_IDENTIFIER, moduleIdentifier.serialize());
		output.put(MODULE_CONFIGURATION_PROPERTIES, moduleConfigurationProperties);
		
		output.put(MODULE_TYPE, moduleType.serialize());
		
		JSONArray calibrationData = new JSONArray();
		for (int i = 0; i < this.calibrationData.size(); i++) {
			calibrationData.put(this.calibrationData.get(i).serialize());
		}
		output.put(MODULE_CALIBRATION_DATA, calibrationData);
		
		return output;
	}
	
	public void insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) throws JSONException {
		moduleType.insertIntoDatabase(knowledgeDBClient);
		
		knowledgeDBClient.executeUpdateQuery(addModule, 
				moduleIdentifier.manufacturer, moduleIdentifier.typeNumber, moduleIdentifier.serialNumber, moduleConfigurationProperties);
		
		for (CalibrationEntry calibrationEntry : calibrationData) {
			calibrationEntry.insertIntoDatabase(knowledgeDBClient);
		}
		
	}
}
