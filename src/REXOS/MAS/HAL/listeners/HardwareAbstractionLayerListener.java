package HAL.listeners;

import java.util.ArrayList;

import HAL.HALTesterClass;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;
import HAL.tasks.ExecutionProcess;
/**
 * A HardwareAbstractionLayerListener listens to events in the {@link HardwareAbstractionLayer}. This interface is usually implemented by the {@link EquipletAgent} or the {@link HALTesterClass}
 * @author Bas Voskuijlen
 *
 */
public interface HardwareAbstractionLayerListener {
	/**
	 * This method is called whenever the status of a HardwareStep is changed. This for example enables the {@link EquipletAgent} to inform {@link ProductAgent} about the progress.
	 * @param state
	 * @param hardwareStepSerialId
	 * @param module
	 * @param hardwareStep
	 */
	public void onProcessStatusChanged(String state, long hardwareStepSerialId, Module module, HardwareStep hardwareStep);
	/**
	 * This method is called when the execution of the {@link HardwareStep}s has finished (e.g. when a {@link ExecutionProcess} finishes)
	 */
	public void onExecutionFinished();
	
	/**
	 * This method is called when the MAST state of the equiplet changes
	 * @param state
	 * @param module
	 */
	public void onEquipletStateChanged(String state, Module module);
	/**
	 * This method is called when the MAST mode of the equiplet changes
	 * @param mode
	 * @param module
	 */
	public void onEquipletModeChanged(String mode, Module module);
	/**
	 * This method is called when the MAST state of the module changes
	 * @param state
	 * @param module
	 */
	public void onModuleStateChanged(String state, Module module);
	/**
	 * This method is called when the MAST mode of the module changes
	 * @param mode
	 * @param module
	 */
	public void onModuleModeChanged(String mode, Module module);
	
	/**
	 * This method is called when the translation of a {@link ProductStep} has succesfully finished.
	 * @param productStep
	 * @param hardwareStep
	 */
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareStep);
	/**
	 * This method is called when the translation of a {@link ProductStep} has failed.
	 * @param productStep
	 */
	public void onIncapableCapabilities(ProductStep productStep);
	
	/**
	 * This method is used by the HAL to retrieve the equiplet name
	 * @return
	 */
	public String getEquipletName();
}
