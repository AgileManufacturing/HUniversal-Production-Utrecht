package HAL.steps;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;

public class OriginPlacement {
	private OriginPlacementType originPlacementType;
	private JSONObject parameters;
	
	public OriginPlacement(OriginPlacementType originPlacementType, JSONObject parameters) {
		this.originPlacementType = originPlacementType;
		this.parameters = parameters;
	}
	
	public OriginPlacementType getIdentifier() {
		return originPlacementType;
	}
	public JSONObject getParameters() {
		return parameters;
	}

	public JSONObject toJSON() {
		JSONObject output = new JSONObject();
		try {
		output.put("originPlacementType", originPlacementType.getName());
		output.put("parameters", parameters);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
		}
		return output;
	}
}
