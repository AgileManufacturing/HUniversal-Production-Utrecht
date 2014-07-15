package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.knowledgedb_client.KnowledgeException;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.OriginPlacement;
import HAL.steps.HardwareStep.HardwareStepStatus;
import HAL.steps.OriginPlacement.OriginIdentifier;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public class StewartGough extends ModuleActor {
	public final static int MAX_ACCELERATION = 4;
	public final static String MOVE_COMMAND_IDENTIFIER = "move";
	public final static String ROTATE_COMMAND_IDENTIFIER = "rotate";
	
	public StewartGough(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException {
		ArrayList<HardwareStep> translatedHardwareSteps = new ArrayList<HardwareStep>();
		JsonObject commandMove = compositeStep.popCommandIdentifier(MOVE_COMMAND_IDENTIFIER).getAsJsonObject();
		//Adjust for maxAcceleration
		int maxAcceleration = MAX_ACCELERATION;
		if (commandMove.get(ModuleActor.MAX_ACCELERATION) != null){
			if (commandMove.get(ModuleActor.MAX_ACCELERATION).getAsInt() < maxAcceleration){
				//New maxAcceleration is set
				maxAcceleration = commandMove.remove(ModuleActor.MAX_ACCELERATION).getAsInt();
			} else {
				//Remove if faster without updating maxAcceleration
				commandMove.remove(ModuleActor.MAX_ACCELERATION);
			}
		}
		
		//Add (in case not defined)
		commandMove.addProperty(ModuleActor.MAX_ACCELERATION, maxAcceleration);
		
		//Add OriginPlacement
		JsonObject parameters = new JsonObject();
		if(compositeStep.getCommand().get(CompositeStep.LOOK_UP)==null){
			parameters.addProperty("ID", "null");
		}
		else{
			parameters.addProperty("ID", compositeStep.getCommand().get(CompositeStep.LOOK_UP).getAsString());
		}
		
		OriginPlacement originPlacement = new OriginPlacement(OriginIdentifier.RELATIVE_TO_IDENTIFIER, parameters);

		
		//Check if hopping is disabled
		boolean forceStraightLine = false;
		if (compositeStep.getCommand().get(FORCE_STRAIGHT_LINE) != null){
			forceStraightLine = compositeStep.getCommand().get(FORCE_STRAIGHT_LINE).getAsBoolean();
		}
		

		JsonObject instructionData = new JsonObject();
		instructionData.add(MOVE, commandMove);
		
		//rotate
		JsonObject commandRotate = compositeStep.popCommandIdentifier(ROTATE_COMMAND_IDENTIFIER).getAsJsonObject();
		instructionData.add(ROTATE, commandRotate);
		
		//Approach
		JsonObject commandApproach = new JsonParser().parse(commandMove.get(APPROACH).toString()).getAsJsonObject();
		JsonObject approachInstructionData = new JsonObject();
		approachInstructionData.add(MOVE, commandApproach);
		approachInstructionData.add(ROTATE, commandRotate);
		approachInstructionData.getAsJsonObject(MOVE).addProperty(ModuleActor.MAX_ACCELERATION, maxAcceleration);
		
		//Entry point
		translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, approachInstructionData, originPlacement));
		System.out.println("approachInstructionData: "+ approachInstructionData);
		System.out.println("instructionData: "+ instructionData);
		//Actual point
		translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, instructionData, originPlacement));
		
		//Placeholder
		translatedHardwareSteps.add(null);

		//Exit point
		translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, approachInstructionData, originPlacement));
		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(new CompositeStep(compositeStep.getProductStep(), compositeStep.getCommand(), compositeStep.getRelativeTo()));
		if (hStep != null){
			translatedHardwareSteps.addAll(hStep);
		}
		
		return translatedHardwareSteps;
	}
}

		