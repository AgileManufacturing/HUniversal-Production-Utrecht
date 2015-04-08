package HAL.listeners;

import java.util.ArrayList;

import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
import HAL.tasks.ExecutionProcess;
import HAL.testerClasses.HALTesterClass;

/**
 * A HardwareAbstractionLayerListener listens to events in the {@link HardwareAbstractionLayer}. This interface is usually implemented by the {@link EquipletAgent} or the {@link HALTesterClass}
 * @author Bas Voskuijlen
 *
 */
public interface HardwareAbstractionLayerListener {
	/**
	 * This method is called whenever the status of a HardwareStep is changed. This for example enables the {@link EquipletAgent} to inform Job about the progress.
	 * 
	 * @param state
	 * @param module
	 * @param hardwareStep
	 */
	public void onProcessStatusChanged(HardwareStepStatus status, Module module, HardwareStep hardwareStep);
	/**
	 * This method is called when the execution of the {@link HardwareStep}s has finished (e.g. when a {@link ExecutionProcess} finishes)
	 */
	public void onExecutionFinished();

	/**
	 * This method is called when the execution of the {@link HardwareStep}s has finished (e.g. when a {@link ExecutionProcess} finishes)
	 */
	public void onExecutionFailed();

	/**
	 * This method is called when the MAST state of the equiplet changes
	 * 
	 * @param state
	 */
	public void onEquipletMachineStateChanged(String state);

	/**
	 * This method is called when the MAST mode of the equiplet changes
	 * 
	 * @param mode
	 */
	public void onEquipletModeChanged(String mode);

	/**
	 * This method is called when the MAST state of the module changes
	 * 
	 * @param state
	 * @param module
	 */
	public void onModuleStateChanged(String state, Module module);

	/**
	 * This method is called when the MAST mode of the module changes
	 * 
	 * @param mode
	 * @param module
	 */
	public void onModuleModeChanged(String mode, Module module);

	/**
	 * This method is called when the translation of a Job has succesfully finished.
	 * 
	 * @param productStep
	 * @param hardwareStep
	 */
	public void onTranslationFinished(String service, JSONObject criteria, ArrayList<HardwareStep> hardwareSteps);

	/**
	 * This method is called when the translation of a Job has failed.
	 * 
	 * @param productStep
	 */
	public void onTranslationFailed(String service, JSONObject criteria);

	/**
	 * This method is used by the HAL to retrieve the equiplet name
	 * 
	 * @return
	 */
	public String getEquipletName();

	/**
	 * This method is called when the MAST mode of the equiplet is supposed to Reload
	 * @param state
	 */
	public void onReloadEquiplet(String state);
}
