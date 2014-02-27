package HAL.listeners;

import HAL.Module;

public interface ModuleListener {
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module);
	public void onModuleStateChanged(String state, Module module);
	public void onModuleModeChanged(String mode, Module module);
}
