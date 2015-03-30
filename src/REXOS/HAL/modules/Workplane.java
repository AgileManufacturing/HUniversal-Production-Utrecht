package HAL.modules;

import HAL.Module;
import HAL.dataTypes.ModuleIdentifier;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;

public class Workplane extends Module {
	public Workplane(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}
}