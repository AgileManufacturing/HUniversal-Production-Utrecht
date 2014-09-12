package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import util.math.Vector3;
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

import org.json.JSONException;
import org.json.JSONObject;

public class Gripper extends ModuleActor {
	// Gonna be loaded from KDB:
	private static final double GRIPPER_SIZE = 46.54;

	private static final String ACTIVATE = "activate";
	private static final String DEACTIVATE = "deactivate";
	private static final String PICK = "pick";
	private static final String PLACE = "place";

	public Gripper(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JSONException {
		ArrayList<HardwareStep> translatedHardwareSteps = new ArrayList<HardwareStep>();
		compositeStep = adjustMoveWithDimensions(compositeStep, new Vector3(0, 0, GRIPPER_SIZE));
		
		//Pop command identifier
		boolean isPick = false, isPlace = false;
		if (compositeStep.getCommand().get(PICK) != null) {
			compositeStep.popCommandIdentifier(PICK);
			isPick = true;
		} else {
			compositeStep.popCommandIdentifier(PLACE);
			isPlace = true;
		}

		translatedHardwareSteps.addAll(forwardCompositeStep(compositeStep));
		int placeholderId = getPlaceholderID(translatedHardwareSteps);

		// Set hardwareSteps
		if (isPick || isPlace) {
			HardwareStep harwareStep;
			JSONObject instructionData = new JSONObject();
			if (isPick) {
				instructionData.put(ACTIVATE, JSONObject.NULL);
				harwareStep = new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, instructionData);
			} else {
				instructionData.put(DEACTIVATE, JSONObject.NULL);
				harwareStep = new HardwareStep(moduleIdentifier, compositeStep, HardwareStepStatus.WAITING, instructionData);
			}
			
			if (placeholderId == -1) {
				translatedHardwareSteps.add(harwareStep);
			} else {
				translatedHardwareSteps.set(placeholderId, harwareStep);
			}
		}

		return translatedHardwareSteps;
	}
}