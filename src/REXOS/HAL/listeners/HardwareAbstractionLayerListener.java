package HAL.listeners;

import generic.Mast;

import java.util.ArrayList;

import org.json.JSONObject;

import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.listeners.EquipletListener.EquipletReloadStatus;
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
	public void onProcessStatusChanged(Module module, HardwareStep hardwareStep, HardwareStepStatus status);
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
	public void onEquipletStateChanged(Mast.State state);

	/**
	 * This method is called when the MAST mode of the equiplet changes
	 * 
	 * @param mode
	 */
	public void onEquipletModeChanged(Mast.Mode mode);

	/**
	 * This method is called when the MAST state of the module changes
	 * 
	 * @param state
	 * @param module
	 */
	public void onModuleStateChanged(Module module, Mast.State state);

	/**
	 * This method is called when the MAST mode of the module changes
	 * 
	 * @param mode
	 * @param module
	 */
	public void onModuleModeChanged(Module module, Mast.Mode mode);

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

	public void onReloadEquipletStatusChanged(EquipletReloadStatus status);
}
