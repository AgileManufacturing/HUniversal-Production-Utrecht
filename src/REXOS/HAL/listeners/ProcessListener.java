package HAL.listeners;

import HAL.Module;
import HAL.steps.HardwareStep;
/**
 * A ProcessListener listens to progress changes of {@link HardwareStep} which are reported by the {@link Module}s. 
 * @author Bas Voskuijlen
 *
 */
public interface ProcessListener {
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module);
}
