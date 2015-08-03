package HAL.listeners;

import HAL.Module;
import HAL.steps.HardwareStep;
import HAL.tasks.ExecutionProcess;

public interface ExecutionProcessListener {
	/**
	 * This method is called whenever the status of a HardwareStep is changed. This for example enables the {@link EquipletAgent} to inform Job about the progress.
	 * 
	 * @param state
	 * @param module
	 * @param hardwareStep
	 */
	public void onProcessStatusChanged(Module module, HardwareStep hardwareStep);
	/**
	 * This method is called when the execution of the {@link HardwareStep}s has finished (e.g. when a {@link ExecutionProcess} finishes)
	 */
	public void onExecutionFinished();

	/**
	 * This method is called when the execution of the {@link HardwareStep}s has finished (e.g. when a {@link ExecutionProcess} finishes)
	 */
	public void onExecutionFailed();

}
