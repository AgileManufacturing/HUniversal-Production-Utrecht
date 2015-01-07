package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import util.log.Logger;
import util.math.Vector3;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.libraries.blackboard_client.data_classes.GeneralMongoException;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;
import HAL.listeners.ModuleListener;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

import org.json.*;

/**
 * @author Lars Veenendaal
 *
 */

public class Gripper extends ModuleActor {
	
	private KnowledgeDBClient knowledgeDBClient;
	
	// Gonna be loaded from KDB:
	private static final String GET_GRIPPER_SIZE = "SELECT * FROM ModuleType WHERE typeNumber = 'gripper_type_A'";
	private static final String ACTIVATE = "activate";
	private static final String DEACTIVATE = "deactivate";
	private static final String PICK = "pick";
	private static final String PLACE = "place";

	public Gripper(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
		knowledgeDBClient = new KnowledgeDBClient();
		getGripperSize();
	}
	
	/*
	 * getGripperSize()
	 * This function gets gripperSize out of the ModuleType Json which is stored on the database.
	 * @returns double gripperSize 
	 */
	public double getGripperSize() {
		double gs = 0.0;
		Row[] resultSet = knowledgeDBClient.executeSelectQuery(	GET_GRIPPER_SIZE );
			String json = resultSet[0].get("moduleTypeProperties").toString();
			JSONTokener token = new JSONTokener(json);
			try {
				JSONObject theProperties = new JSONObject(token);
				gs = theProperties.optDouble("gripperSize");
			} catch (JSONException e1) {
				System.err.println("No gripperSize defined.");
			}
		return gs;
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JSONException {
		Logger.log("Gripper is translating compositeStep: " + compositeStep.toJSON().toString());
		ArrayList<HardwareStep> translatedHardwareSteps = new ArrayList<HardwareStep>();
		compositeStep = adjustMoveWithDimensions(compositeStep, new Vector3(0, 0, getGripperSize()));
		
		//Pop command identifier
		boolean isPick = false, isPlace = false;
		if (compositeStep.getCommand().has(PICK) == true) {
			compositeStep.popCommandIdentifier(PICK);
			isPick = true;
		} else {
			compositeStep.popCommandIdentifier(PLACE);
			isPlace = true;
		}

		translatedHardwareSteps.addAll(forwardCompositeStep(compositeStep));
		int placeholderId = getPlaceholderIndex(translatedHardwareSteps);
		
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
		} else {
			// nothing to do
		}

		return translatedHardwareSteps;
	}
}