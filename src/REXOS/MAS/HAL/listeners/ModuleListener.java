package HAL.listeners;

import HAL.Module;
import HAL.exceptions.HardwareAbstractionLayerProcessException;

public interface ModuleListener {
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module) throws HardwareAbstractionLayerProcessException;
	public void onModuleStateChanged(String state, Module module);
	public void onModuleModeChanged(String mode, Module module);
}
