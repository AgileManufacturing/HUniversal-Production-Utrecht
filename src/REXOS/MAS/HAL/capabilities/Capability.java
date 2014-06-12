package HAL.capabilities;

import java.util.ArrayList;

import libraries.dynamicloader.JarFileLoaderException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.LogSection;
import libraries.utillities.log.Logger;
import HAL.ModuleActor;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public abstract class Capability {
	private String name;
	protected ModuleFactory moduleFactory;
	
	/**
	 * 
	 * @return
	 */
	public String getName() {
		return name;
	}
	
	/**
	 * 
	 * @param moduleFactory
	 * @param capabilityName
	 */
	public Capability(ModuleFactory moduleFactory, String capabilityName) {
		this.moduleFactory = moduleFactory;
		this.name = capabilityName;
	}
	
	/**
	 * 
	 * @param productStep
	 * @return
	 * @throws CapabilityException
	 */
	abstract public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) throws CapabilityException;
	
	protected ArrayList<HardwareStep> translateCompositeStep(ArrayList<ModuleActor> modules, CompositeStep compositeStep) throws CapabilityException {
		ArrayList<CompositeStep> compositeSteps = new ArrayList<CompositeStep>();
		compositeSteps.add(compositeStep);
		return translateCompositeStep(modules, compositeSteps);
	}
	protected ArrayList<HardwareStep> translateCompositeStep(ArrayList<ModuleActor> modules, ArrayList<CompositeStep> compositeSteps) throws CapabilityException {
		for (ModuleActor moduleActor : modules) {
			ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
			try {
				for (CompositeStep compositeStep : compositeSteps) {
					hardwareSteps.addAll(moduleActor.translateCompositeStep(compositeStep));
				}
				return hardwareSteps;
			} catch (ModuleTranslatingException ex) {
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.INFORMATION, "The module tree with bottom module " + moduleActor.getModuleIdentifier() + 
						" was unable to fully translate the composite step: " + ex.getCompositeStep());
			} catch (FactoryException ex) {
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, "The factory was unable to instantiate the module " + moduleActor.getModuleIdentifier() + 
						" continuing with the next one", ex);
			} catch (IllegalArgumentException ex) {
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, "A invalid argument was thrown by the module " + moduleActor.getModuleIdentifier() + 
						" continuing with the next one", ex);
			}
		}
		throw new CapabilityException("No module tree was able to fully translate composite steps: " + compositeSteps);
	}
	
}
