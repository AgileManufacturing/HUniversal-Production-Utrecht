package HAL.listeners;

import HAL.Module;

public interface ModuleListener {
	public void onModuleStateChanged(String state, Module module);
	public void onModuleModeChanged(String mode, Module module);
}
