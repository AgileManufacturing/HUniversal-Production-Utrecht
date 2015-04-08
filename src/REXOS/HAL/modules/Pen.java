package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import org.json.JSONException;

import util.math.Vector3;
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

public class Pen extends ModuleActor {
	private static final String COMMAND_IDENTIFIER = "draw";	
	private static final double PEN_SIZE = 102.6; // in mm
	private static final int MAX_ACCELERATION = 50;

	public Pen(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JSONException {
		ArrayList<HardwareStep> translatedHardwareSteps = new ArrayList<HardwareStep>();
		compositeStep.popCommandIdentifier(COMMAND_IDENTIFIER);
		
		//Adjust the move with the Pen module it's height.
		compositeStep = adjustMoveWithDimensions(compositeStep, new Vector3(0, 0, PEN_SIZE));
		compositeStep.getCommand().put(ModuleActor.MAX_ACCELERATION, MAX_ACCELERATION);
		
		//Translate it's parents composite steps into hardware steps.
		translatedHardwareSteps.addAll(forwardCompositeStep(compositeStep));
		return translatedHardwareSteps;
	}
}