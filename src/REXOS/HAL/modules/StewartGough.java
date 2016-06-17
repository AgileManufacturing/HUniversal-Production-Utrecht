package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.Logger;
import HAL.ModuleActor;
import HAL.dataTypes.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.libraries.blackboard_client.data_classes.GeneralMongoException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.ModuleListener;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
import HAL.steps.OriginPlacement;

public class StewartGough extends ModuleActor {
	public final static double MAX_ACCELERATION = 4;
	public final static String MOVE_COMMAND_IDENTIFIER = "move";
	public final static String ROTATE_COMMAND_IDENTIFIER = "rotate";
	
	public StewartGough(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JSONException {
		Logger.log("Stewart gough is translating compositeStep: " + compositeStep.toJSON().toString());
		ArrayList<HardwareStep> translatedHardwareSteps = new ArrayList<HardwareStep>();
		JSONObject commandMove = compositeStep.popCommandIdentifier(MOVE_COMMAND_IDENTIFIER);
		JSONObject commandRotate = compositeStep.popCommandIdentifier(ROTATE_COMMAND_IDENTIFIER);
		
		//Adjust for maxAcceleration
		double maxAcceleration = MAX_ACCELERATION;
		if (commandMove.get(ModuleActor.MAX_ACCELERATION) != null){
			if (commandMove.getDouble(ModuleActor.MAX_ACCELERATION) < maxAcceleration){
				//New maxAcceleration is set
				maxAcceleration = (double) commandMove.remove(ModuleActor.MAX_ACCELERATION);
			} else {
				//Remove if faster without updating maxAcceleration
				commandMove.remove(ModuleActor.MAX_ACCELERATION);
			}
		}
		
		//Add (in case not defined)
		commandMove.put(ModuleActor.MAX_ACCELERATION, maxAcceleration);
		
		//Add OriginPlacement
		OriginPlacement originPlacement = compositeStep.getOriginPlacement();
		
		//Check if hopping is disabled
		boolean forceStraightLine = false;
		if (compositeStep.getCommand().has(FORCE_STRAIGHT_LINE)){
			forceStraightLine = compositeStep.getCommand().getBoolean(FORCE_STRAIGHT_LINE);
		}
		

		JSONObject instructionData = new JSONObject();
		instructionData.put(MOVE, commandMove);
		instructionData.put(ROTATE, commandRotate);
		if (forceStraightLine){
			//Straight line
			translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, instructionData, originPlacement));	
		} else {
			//Approach
			JSONObject commandApproachMove = commandMove.getJSONObject(APPROACH);
			JSONObject commandApproachRotate = commandRotate.getJSONObject(APPROACH);
			JSONObject approachInstructionData = new JSONObject();
			commandApproachMove.put(MOVE_X, commandApproachMove.getDouble(MOVE_X) + commandMove.getDouble(MOVE_X));
			commandApproachMove.put(MOVE_Y, commandApproachMove.getDouble(MOVE_Y) + commandMove.getDouble(MOVE_Y));
			commandApproachMove.put(MOVE_Z, commandApproachMove.getDouble(MOVE_Z) + commandMove.getDouble(MOVE_Z));
			approachInstructionData.put(MOVE, commandApproachMove);
			commandApproachRotate.put(ROTATION_X, commandApproachRotate.getDouble(ROTATION_X) + commandRotate.getDouble(ROTATION_X));
			commandApproachRotate.put(ROTATION_Y, commandApproachRotate.getDouble(ROTATION_Y) + commandRotate.getDouble(ROTATION_Y));
			commandApproachRotate.put(ROTATION_Z, commandApproachRotate.getDouble(ROTATION_Z) + commandRotate.getDouble(ROTATION_Z));
			approachInstructionData.put(ROTATE, commandApproachRotate);
			
			//Entry point
			translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, approachInstructionData, originPlacement));
			
			//Actual point
			translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, instructionData, originPlacement));
			
			//Placeholder
			translatedHardwareSteps.add(null);

			//Exit point
			translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, approachInstructionData, originPlacement));
		}
		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(new CompositeStep(compositeStep.getService(), compositeStep.getCommand(), compositeStep.getOriginPlacement()));
		if (hStep != null){
			translatedHardwareSteps.addAll(hStep);
		}
		return translatedHardwareSteps;
	}
}

		