package HAL.capabilities;

import java.util.ArrayList;







import libraries.dynamicloader.JarFileLoaderException;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import HAL.Capability;
import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.ModuleActor;
import HAL.ProductStep;
import HAL.exceptions.CapabilityException;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;

/**
 * 
 * @author Aristides Ayala Mendoza
 *
 */
public class PickAndPlace extends Capability {
	private static final int MAX_ACCELERATION = 30;
	
	/**
	 * 
	 * @param moduleFactory
	 */
	public PickAndPlace(ModuleFactory moduleFactory) {
		super(moduleFactory, "PickAndPlace");
	}

	/**
	 * @see Capability#translateProductStep(ProductStep)
	 */
	@Override
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) throws FactoryException, JarFileLoaderException, CapabilityException {
		// TODO Auto-generated method stub
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<>();
		String serviceName = productStep.getService().getName();
		JsonObject productStepCriteria = productStep.getCriteria();
		JsonElement target = productStepCriteria.get("target");
		JsonElement subjects = productStepCriteria.get("subjects");
		
		
		if(serviceName.equals("place")){
			JsonObject pickCommand = new JsonObject();
			JsonObject placeCommand = new JsonObject();
			JsonObject subjectMoveCommand = subjects.getAsJsonArray().get(0).getAsJsonObject().get("move").getAsJsonObject();
			
			subjectMoveCommand.addProperty("maxAcceleration", MAX_ACCELERATION);
			
			pickCommand.addProperty("pick" , "null");
			pickCommand.add("move" ,  subjectMoveCommand);
			
			//pickCommand.add("rotation", subjects.getAsJsonArray().get(0).getAsJsonObject().get("rotation"));
			CompositeStep pick = new CompositeStep(productStep, pickCommand);
			
			subjectMoveCommand = target.getAsJsonObject().get("move").getAsJsonObject();
			subjectMoveCommand.addProperty("maxAcceleration", MAX_ACCELERATION);
			placeCommand.addProperty("place", "null");
			placeCommand.add("move" ,  subjectMoveCommand);
			//placeCommand.add("rotation", target.getAsJsonObject().get("rotation"));
			CompositeStep place = new CompositeStep(productStep, placeCommand);
			

			ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
			
			for (ModuleActor moduleActor : modules) {
				try {
					
					hardwareSteps.addAll(moduleActor.translateCompositeStep(pick));
					hardwareSteps.addAll(moduleActor.translateCompositeStep(place));
					
				} catch (ModuleTranslatingException e) {
					
					throw new CapabilityException(e.toString());
				}
				
			}

		}
		return hardwareSteps;
	}

}
