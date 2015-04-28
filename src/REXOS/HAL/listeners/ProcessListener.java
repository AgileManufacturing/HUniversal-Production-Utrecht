package HAL.listeners;

import HAL.Module;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
/**
 * A ProcessListener listens to progress changes of {@link HardwareStep} which are reported by the {@link Module}s. 
 * @author Bas Voskuijlen
 *
 */
public interface ProcessListener {
	public void onProcessStatusChanged(HardwareStep hardwareStep, HardwareStepStatus status);
}
