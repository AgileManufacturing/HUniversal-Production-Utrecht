package HAL.capabilities;

import java.util.ArrayList;





import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import HAL.Capability;
import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.Module;
import HAL.ModuleActor;
import HAL.ProductStep;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;

public class PickAndPlace extends Capability {

	@Override
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep) {
		// TODO Auto-generated method stub
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<>();
		String serviceName = productStep.getService().getName();
		JsonObject productStepCriteria = productStep.getCriteria();
		JsonElement target = productStepCriteria.get("target");
		JsonElement subjects = productStepCriteria.get("subjects");
		
		
		if(serviceName.equals("pick&place")){
			JsonObject pickCommand = new JsonObject();
			JsonObject placeCommand = new JsonObject();
			
			pickCommand.addProperty("pick" , "null");
			pickCommand.add("move" ,  target.getAsJsonObject().get("move").getAsJsonArray());
			pickCommand.add("rotation", target.getAsJsonObject().get("rotation").getAsJsonArray());
			CompositeStep pick = new CompositeStep(productStep, pickCommand);
			
			placeCommand.addProperty("place", "null");
			placeCommand.add("move" ,  target.getAsJsonObject().get("move").getAsJsonArray());
			placeCommand.add("rotation", target.getAsJsonObject().get("rotation").getAsJsonArray());
			CompositeStep place = new CompositeStep(productStep, placeCommand);
			

			ModuleFactory moduleFactory = null;
			ArrayList<ModuleActor> modules = moduleFactory.getBottomModuleActors();
			for (ModuleActor moduleActor : modules) {
				try {
					hardwareSteps.addAll(moduleActor.translateCompositeStep(pick));
					hardwareSteps.addAll(moduleActor.translateCompositeStep(place));
				} catch (KnowledgeException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (KeyNotFoundException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (ModuleTranslatingException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
			}

		}
		return hardwareSteps;
	}

}
