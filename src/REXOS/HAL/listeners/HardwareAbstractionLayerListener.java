package HAL.listeners;

import HAL.AbstractHardwareAbstractionLayer;
import HAL.testerClasses.HALTesterClass;

/**
 * A HardwareAbstractionLayerListener listens to events in the {@link AbstractHardwareAbstractionLayer}. This interface is usually implemented by the {@link EquipletAgent} or the {@link HALTesterClass}
 * @author Bas Voskuijlen
 *
 */
public interface HardwareAbstractionLayerListener extends MastListener, TranslationProcessListener, ExecutionProcessListener {
}
