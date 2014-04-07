package HAL.capabilities;

import java.util.ArrayList;

import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

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

public class Draw extends Capability {
	
	public Draw(ModuleFactory moduleFactory) {
		super(moduleFactory, "Draw");
	}

	@Override
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep)
			throws CapabilityException, FactoryException, JarFileLoaderException {
		
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<>();
		String serviceName = productStep.getService().getName();
		JsonObject productStepCriteria = productStep.getCriteria();
		JsonElement target = productStepCriteria.get("target");
		JsonElement subjects = productStepCriteria.get("subjects");
		
		if(serviceName.equals("draw")){
			JsonObject moveCommand = new JsonObject();
			JsonObject drawCommand = new JsonObject();
			
			moveCommand.addProperty("move", "null");
			moveCommand.add("move", target.getAsJsonObject().get("move").getAsJsonArray());	
			CompositeStep move = new CompositeStep(productStep, moveCommand);
			
			drawCommand.addProperty("draw", "null");
			moveCommand.add("draw", target.getAsJsonObject().get("draw").getAsJsonArray());
			CompositeStep draw = new CompositeStep(productStep, drawCommand);
		
			ArrayList<ModuleActor> modules = moduleFactory.getBottomModuleActors();
			for (ModuleActor moduleActor : modules) {
				try {
					
					hardwareSteps.addAll(moduleActor.translateCompositeStep(move));
					hardwareSteps.addAll(moduleActor.translateCompositeStep(draw));
					
				} catch (ModuleTranslatingException e) {
					
					throw new CapabilityException(e.toString());
				}
			}
		}
		return hardwareSteps;
	}

}
