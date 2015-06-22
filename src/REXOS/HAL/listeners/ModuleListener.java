package HAL.listeners;

import generic.Mast;
import HAL.dataTypes.ModuleIdentifier;
import HAL.steps.HardwareStep;
/**
 * A ModuleListener listens to changes in the {@link Module}s.
 * A ModuleListener does not listen to changes in the execution of {@link HardwareStep}s, this is done by a {@link ProcessListener}.
 * @author Tommas Bakker
 *
 */
public interface ModuleListener {
	/**
	 * This method is called when the MAST state of the module changes
	 * @param state
	 * @param module
	 */
	public void onModuleStateChanged(ModuleIdentifier module, Mast.State state);
	/**
	 * This method is called when the MAST mode of the module changes
	 * @param mode
	 * @param module
	 */
	public void onModuleModeChanged(ModuleIdentifier module, Mast.Mode mode);
}
