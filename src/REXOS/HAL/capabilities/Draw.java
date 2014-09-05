package HAL.capabilities;

import generic.ProductStep;

import java.util.ArrayList;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.Capability;
import HAL.ModuleActor;
import HAL.exceptions.CapabilityException;
import HAL.factories.ModuleFactory;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.OriginPlacement;
import HAL.steps.OriginPlacementType;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public class Draw extends Capability {	
	public final static String SERVICE_IDENTIFIER = "draw";
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
		JsonObject target = productStep.getCriteria().get(ProductStep.TARGET).getAsJsonObject();
		
		if(productStep.getService().getName().equals(SERVICE_IDENTIFIER) == false) {
			String message = "Recieved a service (" + productStep.getService() + "which is not supported by this capability.";
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, message);
			throw new IllegalArgumentException(message);	
		}
		if(target == null) {
			String message = "Recieved a illegaly formatted product step: " + productStep;
			Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.ERROR, message);
			throw new IllegalArgumentException(message);
		}
		
		// draw
		JsonObject targetMoveCommand = target.get("move").getAsJsonObject();
		
		JsonObject drawCommand = new JsonObject();
		drawCommand.add("draw", null);
		drawCommand.add("move", targetMoveCommand);
		
		JsonObject drawOriginPlacementParameters = new JsonObject();
		drawOriginPlacementParameters.addProperty("identifier", target.get(CompositeStep.IDENTIFIER).getAsString());
		OriginPlacement drawOriginPlacement = new OriginPlacement(OriginPlacementType.RELATIVE_TO_IDENTIFIER, drawOriginPlacementParameters);
		
		CompositeStep draw = new CompositeStep(productStep, drawCommand, drawOriginPlacement);
		Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.DEBUG, "draw: " + draw);
				
		// Translate to hardwareSteps
		ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
		ArrayList<CompositeStep> compositeSteps = new ArrayList<CompositeStep>();
		compositeSteps.add(draw);
			
		ArrayList<HardwareStep> hardwareSteps = translateCompositeStep(modules, compositeSteps);
		Logger.log(LogSection.HAL_CAPABILITIES, LogLevel.INFORMATION, "Translated hardware steps: " + hardwareSteps.toString());
		return hardwareSteps;
	}

}
