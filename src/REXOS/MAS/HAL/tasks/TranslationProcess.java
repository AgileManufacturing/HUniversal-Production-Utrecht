package HAL.tasks;

import java.util.ArrayList;

import libraries.dynamicloader.JarFileLoaderException;
import HAL.Capability;
import HAL.HardwareStep;
import HAL.ProductStep;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.factories.CapabilityFactory;
import HAL.listeners.HardwareAbstractionLayerListener;

public class TranslationProcess implements Runnable{
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private ProductStep productStep;
	private CapabilityFactory capabilityFactory;
	
	public TranslationProcess(HardwareAbstractionLayerListener hardwareAbstractionLayerListener, ProductStep productStep, CapabilityFactory capabilityFactory){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		this.productStep = productStep;
		this.capabilityFactory = capabilityFactory;
	}
	
	@Override
	public void run() {
		System.out.println("Starting translation process");
		ArrayList<Capability> capabilities = null;
		try {
			capabilities = capabilityFactory.getAllSupportedCapabilities();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		if (capabilities != null){
			int numCapabilities = capabilities.size();
			if (numCapabilities == 0){
				hardwareAbstractionLayerListener.onIncapableCapabilities(productStep);
			}
			else {
				ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
				
				for (int i=0;i<numCapabilities;i++){
					try {
						hardwareSteps.addAll(capabilities.get(i).translateProductStep(productStep));
						
					} catch (CapabilityException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} catch (FactoryException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} catch (JarFileLoaderException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				
				for (HardwareStep hardwareStep : hardwareSteps) {
					System.out.println(hardwareStep.getCommand().toString()+"\n");
				} 
				
				hardwareAbstractionLayerListener.onTranslationFinished(productStep, hardwareSteps);			
			}
		}
		else {
			hardwareAbstractionLayerListener.onIncapableCapabilities(productStep);			
		}
	}
}
