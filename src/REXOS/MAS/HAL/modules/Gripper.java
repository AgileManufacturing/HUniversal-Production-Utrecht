package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.knowledgedb_client.KnowledgeException;
import libraries.math.Vector3;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

public class Gripper extends ModuleActor {
	//Gonna be loaded from KDB:
	private static final double GRIPPER_SIZE = 46.54; 
	
	private static final String ACTIVATE = "activate";
	private static final String DEACTIVATE = "deactivate";
	private static final String PICK = "pick";
	private static final String PLACE = "place";

	
	public Gripper(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		JsonObject jsonCommand = compositeStep.getCommand();
		JsonObject command = jsonCommand.remove(HardwareStep.COMMAND).getAsJsonObject();
		
		command = adjustMoveWithDimensions(command, new Vector3(0, 0, GRIPPER_SIZE));
		JsonElement pick = command.remove(PICK);
		JsonElement place = command.remove(PLACE);
		command.addProperty("forceStraightLine", false);
		
		jsonCommand.add(HardwareStep.COMMAND, command);
		
		compositeStep = new CompositeStep(compositeStep.getProductStep(),jsonCommand);		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(compositeStep);
		int placeholderId = -1;
		if (hStep != null){
			hardwareSteps.addAll(hStep);
			for (int i=0;i<hStep.size();i++){
				if (hStep.get(i) == null){
					placeholderId = i;
					i = hStep.size();
				}
			}
		}
		
		//Set hardwareSteps
		if (pick != null || place != null){
			JsonObject hardwareCommand = new JsonObject();
			if (pick != null){
				JsonObject instructionData = new JsonObject();
				instructionData.addProperty(HardwareStep.COMMAND, ACTIVATE);
				hardwareCommand.add("instructionData", instructionData);
			}
			else{
				JsonObject instructionData = new JsonObject();
				instructionData.addProperty(HardwareStep.COMMAND, DEACTIVATE);
				hardwareCommand.add("instructionData", instructionData);
			}
			hardwareCommand.add("moduleIdentifier",moduleIdentifier.getAsJSON());
			hardwareCommand.addProperty("status","WAITING");
			
			if (placeholderId == -1){
				hardwareSteps.add(new HardwareStep(compositeStep,hardwareCommand,moduleIdentifier));
			}
			else {
				hardwareSteps.set(placeholderId, new HardwareStep(compositeStep,hardwareCommand,moduleIdentifier));
			}
		}
		
		return hardwareSteps;
	}
}
