package HAL.capabilities;

import generic.ProductStep;

import java.util.ArrayList;

import libraries.log.LogLevel;
import libraries.log.LogSection;
import libraries.log.Logger;
import HAL.Capability;
import HAL.ModuleActor;
import HAL.exceptions.CapabilityException;
import HAL.factories.ModuleFactory;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;

import com.google.gson.JsonObject;

/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public class Draw extends Capability {	
	/**
	 * 
	 * @param moduleFactory
	 */
	public Draw(ModuleFactory moduleFactory) {
		super(moduleFactory, "Draw");
	}

	/**
	 * @see Capability#translateProductStep(ProductStep)
	 */
	@Override
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) throws CapabilityException {
		
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<>();
		String serviceName = productStep.getService().getName();
		JsonObject productStepCriteria = productStep.getCriteria();
		JsonObject target = productStepCriteria.get(ProductStep.TARGET).getAsJsonObject();
		
		if(serviceName.equals("draw") && target != null){
			JsonObject moveCommand = target.get("move").getAsJsonObject();

			JsonObject command = new JsonObject();
			command.addProperty("draw", "null");
			command.add("move", moveCommand);

			JsonObject relativeTo = new JsonObject();
			relativeTo.add("identifier", target.get("identifier"));

			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "command: " + command + ", relativeTo: " + relativeTo);
			
			ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
			CompositeStep draw = new CompositeStep(productStep, command, relativeTo);
			hardwareSteps.addAll(translateCompositeStep(modules, draw));
			
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.INFORMATION, "Translated hardware steps: " + hardwareSteps.toString());
		}
		else
			throw new CapabilityException("Invalid service type or no target specified", null);
		return hardwareSteps;
	}

}
