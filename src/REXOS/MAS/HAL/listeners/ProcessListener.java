package HAL.listeners;

import HAL.Module;
import HAL.exceptions.HardwareAbstractionLayerProcessException;

public interface ProcessListener {
	public void onProcessStatusChanged(String status, long hardwareStepSerialId, Module module) throws HardwareAbstractionLayerProcessException;
}
