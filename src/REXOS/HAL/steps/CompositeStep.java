package HAL.steps;

import HAL.Capability;
import HAL.Module;
import HAL.exceptions.ModuleTranslatingException;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;

/**
 * A CompositeStep is a step composed of multiple abstract {@link HardwareStep}s.
 * CompositeSteps are generated by {@link Capability} and interpeted by {@link Module}s.
 * 
 * @author Bas Voskuijlen
 *
 */
public class CompositeStep {
	private String service;
	private JSONObject command;
	private OriginPlacement originPlacement;

	public static final String IDENTIFIER = "identifier";
	public static final String RELATIVE_TO = "relativeTo";

	public CompositeStep(String service, JSONObject command, OriginPlacement originPlacement) {
		this.command = command;
		this.originPlacement = originPlacement;
		this.service = service;
	}

	public String getService() {
		return service;
	}

	public JSONObject getCommand() {
		return this.command;
	}

	public OriginPlacement getOriginPlacement() {
		return this.originPlacement;
	}

	public JSONObject popCommandIdentifier(String identifier) throws ModuleTranslatingException {
		if (command.has(identifier) == false) {
			throw new RuntimeException("Module didn't find a \"" + identifier + "\" key in CompositeStep command: " + command);
		}
		JSONObject mutation = command.optJSONObject(identifier);
		command.remove(identifier);
		return mutation;
	}

	public JSONObject toJSON() {
		JSONObject returnValue = new JSONObject();
		try {
			returnValue.put("command", command);
			returnValue.put("originPlacement", originPlacement.toJSON());
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
		}
		return returnValue;
	}

	public CompositeStep clone() {
		try {
			return new CompositeStep(service, new JSONObject(command.toString()), originPlacement.clone());
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
			return null;
		}
	}
	public String toString() {
		return toJSON().toString();
	}
}
