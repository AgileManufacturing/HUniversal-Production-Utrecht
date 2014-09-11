package HAL.steps;

import com.google.gson.JsonObject;

public class OriginPlacement {
	private OriginPlacementType originPlacementType;
	private JsonObject parameters;
	
	public OriginPlacement(OriginPlacementType originPlacementType, JsonObject parameters) {
		this.originPlacementType = originPlacementType;
		this.parameters = parameters;
	}
	
	public OriginPlacementType getIdentifier() {
		return originPlacementType;
	}
	public JsonObject getParameters() {
		return parameters;
	}

	public JsonObject toJSON() {
		JsonObject output = new JsonObject();
		output.addProperty("originPlacementType", originPlacementType.getName());
		output.add("parameters", parameters);
		return output;
	}
}
