package HAL.listeners;

import HAL.Module;
import HAL.steps.HardwareStep;
/**
 * A ModuleListener listens to changes in the {@link Module}s.
 * A ModuleListener does not listen to changes in the execution of {@link HardwareStep}s, this is done by a {@link ProcessListener}.
 * @author Tommas Bakker
 *
 */
public interface ViolationListener {
	public enum ViolationType {
		COLLISTION, JOINT, ACCELERATION, UNDEFINED
	}
	
	public void onViolationOccured(ViolationType violationType, String message);
}
