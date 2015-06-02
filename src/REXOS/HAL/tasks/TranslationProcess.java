package HAL.tasks;

import java.util.ArrayList;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.Capability;
import HAL.exceptions.CapabilityException;
import HAL.factories.CapabilityFactory;
import HAL.listeners.TranslationProcessListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

/**
 * The thread that manages the translation of ProductSteps.
 * Although translation usually does not take very long, complex cases may take some milliseconds. This will jam the {@link EquipletAgent} which is considered to be undesirable.
 * @author Bas Voskuijlen
 *
 */
public class TranslationProcess implements Runnable{
	private TranslationProcessListener listener;
	private ProductStep step;
	private CapabilityFactory capabilityFactory;
	/**
	 * Constructs the TranslationProcess but does NOT start it.
	 * @param hardwareAbstractionLayerListener
	 * @param service
	 * @param criteria
	 * @param capabilityFactory
	 */
	public TranslationProcess(TranslationProcessListener listener, ProductStep step, CapabilityFactory capabilityFactory){
		this.listener = listener;
		this.step = step;
		this.capabilityFactory = capabilityFactory;
	}
	
	/**
	 * This method will translate the Job
	 * This method returns after the ProductStep has been translated (complex cases may take some milliseconds) and therefore should be called asynchronous. This should be done with the start method.
	 */
	@Override
	public void run() {
		Logger.log(LogSection.HAL_TRANSLATION, LogLevel.INFORMATION, "Translation process started with job: "
				+ step.getService() + " with criteria " + step.getCriteria());
		
		ArrayList<Capability> capabilities = capabilityFactory.getCapabilitiesForService(step.getService());
		
		if (capabilities.size() == 0) {
			Logger.log(LogSection.HAL_TRANSLATION, LogLevel.ERROR, "This equiplet has no capabilities for the service in job: "
					+ step.getService() + " with criteria " + step.getCriteria() + 
					". This should never happen as the equiplet should know which services are supported.");
			listener.onTranslationFailed(step);
		} else {
			ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
			
			for (int i = 0; i < capabilities.size(); i++) {
				try{
					ArrayList<HardwareStep> translatedSteps = new ArrayList<HardwareStep>();
					translatedSteps = capabilities.get(i).translateProductStep(step.getService(), step.getCriteria());
					
					for (int j = 0; j < translatedSteps.size(); j++){
						// a translated hardware step from a module might be null if it is a place holder, we skip these place holders
						if (translatedSteps.get(j) != null) hardwareSteps.add(translatedSteps.get(j));
					}
					listener.onTranslationFinished(step, hardwareSteps);
					return;
				} catch (CapabilityException ex) {
					Logger.log(LogSection.HAL_TRANSLATION, LogLevel.INFORMATION, "Capability " + capabilities.get(i).getName() + 
							" failed to translate job: " + step.getService() + " with criteria " + step.getCriteria());
				}
			}
			Logger.log(LogSection.HAL_TRANSLATION, LogLevel.NOTIFICATION, "No capability was able to translated job: "
						+ step.getService() + " with criteria " + step.getCriteria());
			listener.onTranslationFailed(step);
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
