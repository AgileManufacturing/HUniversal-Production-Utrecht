package HAL.modules;

import HAL.Module;
import HAL.ModuleIdentifier;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;

public class DummyModuleB extends Module {
	public DummyModuleB(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}
}