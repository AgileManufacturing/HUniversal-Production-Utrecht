package HAL.steps;

import com.google.gson.JsonObject;

public class OriginPlacement {
	public enum OriginIdentifier {
	    RELATIVE_TO_IDENTIFIER,
	    RELATIVE_TO_CURRENT_POSITION,
	    RELATIVE_TO_MODULE_ORIGIN
	}
	
	private OriginIdentifier identifier;
	private JsonObject parameters;
	
	public OriginPlacement(OriginIdentifier identifier, JsonObject parameters) {
		this.identifier = identifier;
		this.parameters = parameters;
	}
	
	public OriginIdentifier getIdentifier() {
		return identifier;
	}
	public JsonObject getParameters() {
		return parameters;
	}
}
