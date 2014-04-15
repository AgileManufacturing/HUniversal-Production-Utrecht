package HAL.listeners;

import java.util.ArrayList;

import HAL.ProductStep;
import HAL.HardwareStep;
import HAL.Module;

public interface HardwareAbstractionLayerListener {
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module, HardwareStep hardwareStep);
	public void onExecutionFinished();
	public void onModuleStateChanged(String state, Module module);
	public void onModuleModeChanged(String mode, Module module);
	
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareStep);
	public void onIncapableCapabilities(ProductStep productStep);
}
