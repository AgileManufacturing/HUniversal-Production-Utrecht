package HAL.capabilities;

import java.util.ArrayList;

import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

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
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep)
			throws CapabilityException, FactoryException, JarFileLoaderException {
		
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<>();
		String serviceName = productStep.getService().getName();
		JsonObject productStepCriteria = productStep.getCriteria();
		JsonObject target = productStepCriteria.get("target").getAsJsonObject();
		JsonElement subjects = productStepCriteria.get("subjects");
		
		if(serviceName.equals("draw") && target != null){
			JsonObject moveCommand = target.get("move").getAsJsonObject();

			JsonObject command = new JsonObject();
			command.addProperty("draw", "null");
			command.add("move", moveCommand);

			JsonObject jsonCommand = new JsonObject();
			jsonCommand.add("command", command);

			jsonCommand.addProperty("look_up", target.get("identifier").getAsString());

			System.out.println("command" + jsonCommand);
			
			ArrayList<ModuleActor> modules = moduleFactory.getBottomModulesForFunctionalModuleTree(this, 1);
			for (ModuleActor moduleActor : modules) {
				JsonObject a = new JsonParser().parse(jsonCommand.toString()).getAsJsonObject();
				CompositeStep draw = new CompositeStep(productStep, a);
				try {
					
					hardwareSteps.addAll(moduleActor.translateCompositeStep(draw));
					return hardwareSteps;
				} catch (ModuleTranslatingException ex) {
					throw new CapabilityException(ex.toString(), ex);
				}
			}
		}
		return hardwareSteps;
	}

}
