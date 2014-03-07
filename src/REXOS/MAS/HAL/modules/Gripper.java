package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.blackboard_client.data_classes.InvalidJSONException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.exceptions.ModuleExecutingException;
import HAL.exceptions.ModuleTranslatingException;

public class Gripper extends ModuleActor {
	
	/*
	 * JSONObject obj = new JSONObject("{interests : [{interestKey:Dogs}, {interestKey:Cats}]}");

List<String> list = new ArrayList<String>();
JSONArray array = obj.getJSONArray("interests");
for(int i = 0 ; i < array.length() ; i++){
    list.add(array.getJSONObject(i).getString("interestKey"));
}
	 */
	
	
	
	//Gonna be loaded from KDB:
	private static final double GRIPPER_SIZE = 18.24;

	
	
	public Gripper(ModuleIdentifier moduleIdentifier) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier);
	}

	@Override
	public void executeHardwareStep(HardwareStep hardwareStep) throws ModuleExecutingException {
		JsonObject command = hardwareStep.getCommand();
		String moduleCommand = command.get("module_command").getAsString();
		if (moduleCommand.equals("activate_gripper") || moduleCommand.equals("deactivate_gripper")){
			try {
				mongoClient.insertDocument(command.toString());
			} catch (InvalidJSONException e) {
				throw new ModuleExecutingException("Executing invalid JSON");
			} catch (InvalidDBNamespaceException e) {
				throw new ModuleExecutingException("Executing invalid DBNamespace");
			} catch (GeneralMongoException e) {
				throw new ModuleExecutingException("General mongo exception while trying to execute");
			}
		}
		else {
			//Impossible
		}
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws KnowledgeException, KeyNotFoundException, ModuleTranslatingException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject command = compositeStep.getCommand();
		JsonElement pick = command.remove("pick");
		JsonElement place = command.remove("place");
		JsonElement move = command.remove("move");
		//command.addProperty("move", move.getAsDouble());
		//Adjustments for gripper dimentions
		
		
		//TODO adjust for dimentions of gripper
		
		compositeStep = new CompositeStep(compositeStep.getProductStep(),command);
		
		ModuleActor moduleActor = (ModuleActor) getParentModule();
		if (moduleActor != null){
			hardwareSteps.addAll(moduleActor.translateCompositeStep(compositeStep));
		}
		else { //Root module, no more parents			
			//Check for remaining commands, then not capible
			JsonArray array = command.getAsJsonArray();
			if (array.size() > 0){
				throw new ModuleTranslatingException("The compositestep isn't completely empty.");
			}
		}
		
		//Set hardwareSteps
		if (pick != null || place != null){
			JsonObject hardwareCommand = new JsonObject();
			if (pick != null){
				hardwareCommand.addProperty("module_command", "activate_gripper");
				hardwareCommand.addProperty("command", "activate");				
			}
			else{
				hardwareCommand.addProperty("module_command", "deactivate_gripper");
				hardwareCommand.addProperty("command", "deactivate");				
			}
			hardwareCommand.addProperty("destination", "gripper");
			hardwareCommand.addProperty("look_up", "NULL");
			
			hardwareSteps.add(new HardwareStep(compositeStep,hardwareCommand,moduleIdentifier));
		}
		
		
		return hardwareSteps;
	}
}
