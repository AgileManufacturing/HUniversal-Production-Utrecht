package HAL.steps;

import HAL.Module;
import HAL.dataTypes.ModuleIdentifier;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;

/**
 * A HardwareStep is a step that is completely translated and can be processed by the ROS node corresponding to this HardwareStep.
 * HardwareSteps are generated by {@link Module}s and interpreted by ROS.
 * @author Bas Voskuijlen
 *
 */
public class HardwareStep implements Cloneable{
	public enum HardwareStepStatus {
		DONE,
		FAILED,
		IN_PROGRESS,
		WAITING
	}
	
	public static final String ORIGIN_PLACEMENT = "originPlacement";
	public static final String INSTRUCTION_DATA = "instructionData";
	public static final String PAYLOAD = "payload";
	public static final String STATUS = "status";
	public static final String MODULE_IDENTIFIER = "moduleIdentifier";
	
	/*
	public static final String MODULE_IDENTIFIER_MANUFACTURER = "manufacturer";
	public static final String MODULE_IDENTIFIER_TYPE_NUMBER = "typeNumber";
	public static final String MODULE_IDENTIFIER_SERIAL_NUMBER = "serialNumber";
*/

	private ModuleIdentifier moduleIdentifier;
	private CompositeStep compositeStep;
	private HardwareStepStatus hardwareStepStatus;

	private JSONObject instructionData;
	private OriginPlacement originPlacement;

	
	public HardwareStep(ModuleIdentifier moduleIdentifier, CompositeStep compositeStep, HardwareStepStatus hardwareStepStatus, JSONObject instructionData) {
		javaIsGayConstructor(moduleIdentifier, compositeStep, hardwareStepStatus, instructionData, null);
	}
	public HardwareStep(ModuleIdentifier moduleIdentifier, CompositeStep compositeStep, HardwareStepStatus hardwareStepStatus, JSONObject instructionData, OriginPlacement originPlacement) {
		javaIsGayConstructor(moduleIdentifier, compositeStep, hardwareStepStatus, instructionData, originPlacement);
	}
	
	private void javaIsGayConstructor(ModuleIdentifier moduleIdentifier, CompositeStep compositeStep, HardwareStepStatus hardwareStepStatus, JSONObject instructionData, OriginPlacement originPlacement) {
		this.moduleIdentifier = moduleIdentifier;
		this.compositeStep = compositeStep;
		this.instructionData = instructionData;
		this.hardwareStepStatus = hardwareStepStatus;
		this.originPlacement = originPlacement;
	}
	
	/*public HardwareStep(CompositeStep compositeStep, JSONObject rosCommand, ModuleIdentifier moduleIdentifier) {
		this.moduleIdentifier = moduleIdentifier;
		//this.rosCommand = rosCommand;
		this.compositeStep = compositeStep;
	}*/
	
	

	public ModuleIdentifier getModuleIdentifier() {
		return this.moduleIdentifier;
	}
	public CompositeStep getCompositeStep() {
		return this.compositeStep;
	}
	public HardwareStepStatus getHardwareStepStatus() {
		return this.hardwareStepStatus;
	}
	public JSONObject getInstructionData() {
		return this.instructionData;
	}
	public OriginPlacement getOriginPlacement() {
		return this.originPlacement;
	}
	
	
	public JSONObject toJSON() {
		JSONObject returnValue = new JSONObject();
		try {
			returnValue.put(MODULE_IDENTIFIER, moduleIdentifier.serialize());
			returnValue.put(STATUS, hardwareStepStatus.toString());
			returnValue.put(INSTRUCTION_DATA, instructionData);
			if(originPlacement != null) {
				returnValue.put(ORIGIN_PLACEMENT, originPlacement.toJSON());
			} else {
				returnValue.put(ORIGIN_PLACEMENT, JSONObject.NULL);
			}
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
		}
		return returnValue;
	}
	
	public String toString() {
		return toJSON().toString();
	}
	public HardwareStep clone() {
		try {
			return new HardwareStep(moduleIdentifier, compositeStep, hardwareStepStatus, new JSONObject(instructionData.toString()), originPlacement);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occurred which is considered to be impossible", ex);
			return null;
		}
	}
}
