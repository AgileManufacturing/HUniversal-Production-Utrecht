package HAL;

import java.util.ArrayList;

import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.tasks.ExecutionProcess;
import HAL.tasks.TranslationProcess;

public class HardwareAbstractionLayer{
	private CapabilityFactory capabilityFactory;
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	
	public HardwareAbstractionLayer(HardwareAbstractionLayerListener hardwareAbstractionLayerListener){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		capabilityFactory = new CapabilityFactory();
	}
	
	public void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps){
		ExecutionProcess executionProcess = new ExecutionProcess(this.hardwareAbstractionLayerListener, hardwareSteps);
		executionProcess.run();
	}
	public void translateProductStep(ProductStep productStep){
		TranslationProcess translationProcess = new TranslationProcess(this.hardwareAbstractionLayerListener, productStep);
		translationProcess.run();
	}
	public ArrayList<Capability> getAllCapabilities(){
		return capabilityFactory.getAllCapabilities();
	}
}
