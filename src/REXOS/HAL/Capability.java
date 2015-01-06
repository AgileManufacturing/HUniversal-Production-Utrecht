package HAL;

import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;

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
	abstract public ArrayList<HardwareStep> translateProductStep(String service, JSONObject criteria) throws CapabilityException;
	
	protected ArrayList<HardwareStep> translateCompositeStep(ArrayList<ModuleActor> modules, CompositeStep compositeStep) throws CapabilityException, JSONException {
		ArrayList<CompositeStep> compositeSteps = new ArrayList<CompositeStep>();
		compositeSteps.add(compositeStep);
		return translateCompositeStep(modules, compositeSteps);
	}
	protected ArrayList<HardwareStep> translateCompositeStep(ArrayList<ModuleActor> modules, ArrayList<CompositeStep> compositeSteps) throws CapabilityException, JSONException {
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
				Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, "A invalid argument exception was thrown by the module " + moduleActor.getModuleIdentifier() + 
						" continuing with the next one", ex);
			}
		}
		throw new CapabilityException("No module tree was able to fully translate composite steps: " + compositeSteps);
	}
	
}
