package HAL.capabilities;

import java.util.ArrayList;

import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.factories.ModuleFactory;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;
import libraries.dynamicloader.JarFileLoaderException;

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
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 */
	abstract public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) 
			throws CapabilityException, FactoryException, JarFileLoaderException;
	
}
