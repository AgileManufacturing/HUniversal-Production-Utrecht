package HAL.tasks;

import java.util.ArrayList;

import agents.equiplet_agent.EquipletAgent;
import libraries.dynamicloader.JarFileLoaderException;
import HAL.capabilities.Capability;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.factories.CapabilityFactory;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;
/**
 * The thread that manages the translation of ProductSteps.
 * Although translation usually does not take very long, complex cases may take some milliseconds. This will jam the {@link EquipletAgent} which is considered to be undesirable.
 * @author Bas Voskuijlen
 *
 */
public class TranslationProcess implements Runnable{
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private ProductStep productStep;
	private CapabilityFactory capabilityFactory;
	/**
	 * Constructs the TranslationProcess but does NOT start it.
	 * @param hardwareAbstractionLayerListener
	 * @param productStep
	 * @param capabilityFactory
	 */
	public TranslationProcess(HardwareAbstractionLayerListener hardwareAbstractionLayerListener, ProductStep productStep, CapabilityFactory capabilityFactory){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		this.productStep = productStep;
		this.capabilityFactory = capabilityFactory;
	}
	
	/**
	 * This method will translate the {@link ProductStep}
	 * This method returns after the ProductStep has been translated (complex cases may take some milliseconds) and therefore should be called asynchronous. This should be done with the start method.
	 */
	@Override
	public void run() {
		System.out.println("Starting translation process");
		ArrayList<Capability> capabilities = null;
		try {
			capabilities = capabilityFactory.getCapabilitiesForService(productStep.getService());
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
					System.out.println(hardwareStep.getRosCommand().toString()+"\n");
				} 
				
				hardwareAbstractionLayerListener.onTranslationFinished(productStep, hardwareSteps);			
			}
		}
		else {
			hardwareAbstractionLayerListener.onIncapableCapabilities(productStep);			
		}
	}

	/**
	 * This method starts the TranslationProcess asynchronously.
	 */
	public void start() {
		Thread t = new Thread(this);
		t.start();
	}
}
