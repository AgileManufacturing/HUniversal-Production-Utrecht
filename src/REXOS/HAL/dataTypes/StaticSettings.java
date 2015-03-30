package HAL.dataTypes;

import java.io.Serializable;
import java.text.ParseException;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class StaticSettings implements Serializable{
	private static final long serialVersionUID = -3990335286891678647L;
	
	public static final String MODULE_IDENTIFIER = "moduleIdentifier";
	public static final String MODULE_CONFIGURATION_PROPERTIES = "properties";
	public static final String MODULE_TYPE = "type";
	public static final String MODULE_CALIBRATION_DATA = "calibrationData";

	public ModuleIdentifier moduleIdentifier;
	public JSONObject moduleConfigurationProperties;
	public ModuleType moduleType;
	public ArrayList<CalibrationEntry> calibrationData;
	
	public StaticSettings() {
		calibrationData = new ArrayList<CalibrationEntry>();
		moduleType = new ModuleType();
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
		
		output.put(MODULE_TYPE, moduleType.serialize());
		
		JSONArray calibrationData = new JSONArray();
		for (int i = 0; i < this.calibrationData.size(); i++) {
			calibrationData.put(this.calibrationData.get(i).serialize());
		}
		output.put(MODULE_CALIBRATION_DATA, calibrationData);
		
		return output;
	}
}
