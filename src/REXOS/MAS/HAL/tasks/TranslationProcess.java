package HAL.tasks;

import java.util.ArrayList;

import HAL.Capability;
import HAL.CapabilityFactory;
import HAL.HardwareStep;
import HAL.ProductStep;
import HAL.listeners.HardwareAbstractionLayerListener;

public class TranslationProcess implements Runnable{
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private ProductStep productStep;
	
	public TranslationProcess(HardwareAbstractionLayerListener hardwareAbstractionLayerListener, ProductStep productStep){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		this.productStep = productStep;
	}
	
	@Override
	public void run() {
		CapabilityFactory capabilityFactory = new CapabilityFactory();
		ArrayList<Capability> capabilities = capabilityFactory.getAllCapabilities();
		
		if (capabilities != null){
			int numCapabilities = capabilities.size();
			if (numCapabilities == 0){
				hardwareAbstractionLayerListener.onIncapableCapabilities(productStep);
			}
			else {
				ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
				
				for (int i=0;i<numCapabilities;i++){
					hardwareSteps.addAll(capabilities.get(i).translateProductStep(productStep));
				}			
				
				hardwareAbstractionLayerListener.onTranslationFinished(productStep, hardwareSteps);			
			}
		}
		else {
			hardwareAbstractionLayerListener.onIncapableCapabilities(productStep);			
		}
	}
}
