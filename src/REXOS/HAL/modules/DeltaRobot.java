package HAL.modules;

import java.util.ArrayList;

import util.log.Logger;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
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

import org.json.JSONException;
import org.json.JSONObject;

public class DeltaRobot extends ModuleActor {
	public final static double MAX_ACCELERATION = 50;
	public final static String COMMAND_IDENTIFIER = "move";
	
	public DeltaRobot(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JSONException {
		Logger.log("Delta robot is translating compositeStep: " + compositeStep.toJSON().toString());
		ArrayList<HardwareStep> translatedHardwareSteps = new ArrayList<HardwareStep>();
		JSONObject commandMove = compositeStep.popCommandIdentifier(COMMAND_IDENTIFIER);
		
		//Adjust for maxAcceleration
		double maxAcceleration = MAX_ACCELERATION;
		if (commandMove.has(ModuleActor.MAX_ACCELERATION) == true){
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
		if (forceStraightLine){
			//Straight line
			translatedHardwareSteps.add(new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, instructionData, originPlacement));	
		} else {
			//Approach
			JSONObject commandApproach = commandMove.getJSONObject(APPROACH);
			commandApproach.put(MOVE_X, commandApproach.getDouble(MOVE_X) + commandMove.getDouble(MOVE_X));
			commandApproach.put(MOVE_Y, commandApproach.getDouble(MOVE_Y) + commandMove.getDouble(MOVE_Y));
			commandApproach.put(MOVE_Z, commandApproach.getDouble(MOVE_Z) + commandMove.getDouble(MOVE_Z));
			
			JSONObject approachInstructionData = new JSONObject();
			approachInstructionData.put(MOVE, commandApproach);
			
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
