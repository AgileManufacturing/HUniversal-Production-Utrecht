package HAL.listeners;

import java.util.ArrayList;

import HAL.Module;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public interface HardwareAbstractionLayerListener {
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module, HardwareStep hardwareStep);
	public void onModuleStateChanged(String state, Module module);
	public void onModuleModeChanged(String mode, Module module);
	
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareStep);
	public void onIncapableCapabilities(ProductStep productStep);
	public void onExecutionFinished();
	
	public String getEquipletName();
}
