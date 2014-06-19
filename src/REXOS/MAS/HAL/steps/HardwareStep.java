package HAL.steps;

import HAL.Module;
import HAL.ModuleIdentifier;

import com.google.gson.JsonObject;

/**
 * A HardwareStep is a step that is completely translated and can be processed by the ROS node corresponding to this HardwareStep.
 * HardwareSteps are generated by {@link Module}s and interpeted by ROS.
 * @author Bas Voskuijlen
 *
 */
public class HardwareStep {
	public enum HardwareStepStatus {
		DONE,
		FAILED,
		IN_PROGRESS,
		WAITING
	}
	
	public static final String COMMAND = "command";
	public static final String LOOK_UP = "look_up";
	public static final String INSTRUCTION_DATA = "instructionData";
	public static final String PAYLOAD = "payload";
	public static final String STATUS = "status";
	public static final String MODULE_IDENTIFIER = "moduleIdentifier";
	

	private ModuleIdentifier moduleIdentifier;
	private CompositeStep compositeStep;
	private HardwareStepStatus hardwareStepStatus;

	private JsonObject instructionData;
	private OriginPlacement originPlacement;

	
	public HardwareStep(ModuleIdentifier moduleIdentifier, CompositeStep compositeStep, HardwareStepStatus hardwareStepStatus, JsonObject instructionData) {
		javaIsGayConstructor(moduleIdentifier, compositeStep, hardwareStepStatus, instructionData, null);
	}
	public HardwareStep(ModuleIdentifier moduleIdentifier, CompositeStep compositeStep, HardwareStepStatus hardwareStepStatus, JsonObject instructionData, OriginPlacement originPlacement) {
		javaIsGayConstructor(moduleIdentifier, compositeStep, hardwareStepStatus, instructionData, originPlacement);
	}
	
	private void javaIsGayConstructor(ModuleIdentifier moduleIdentifier, CompositeStep compositeStep, HardwareStepStatus hardwareStepStatus, JsonObject instructionData, OriginPlacement originPlacement) {
		this.moduleIdentifier = moduleIdentifier;
		this.compositeStep = compositeStep;
		this.instructionData = instructionData;
		this.hardwareStepStatus = hardwareStepStatus;
		this.originPlacement = originPlacement;
	}
	
	
	
	/*public HardwareStep(CompositeStep compositeStep, JsonObject rosCommand, ModuleIdentifier moduleIdentifier) {
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
	public JsonObject getInstructionData() {
		return this.instructionData;
	}
	public OriginPlacement getOriginPlacement() {
		return this.originPlacement;
	}
	
	
	public JsonObject toJSON() {
		
		
		
		return null;
	}
}
