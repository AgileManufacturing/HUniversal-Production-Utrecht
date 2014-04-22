package HAL.listeners;

import HAL.Module;
import HAL.exceptions.HardwareAbstractionLayerProcessException;

public interface ProcessListener {
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module) throws HardwareAbstractionLayerProcessException;
}
