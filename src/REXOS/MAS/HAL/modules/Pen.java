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

import com.google.gson.JsonObject;

public class Pen extends ModuleActor {
	private static final String COMMAND_IDENTIFIER = "draw";
	
	private static final double PEN_SIZE = 102.6; // in mm
	private static final int MAX_ACCELERATION = 50;

	public Pen(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException {
		compositeStep.popCommandIdentifier(COMMAND_IDENTIFIER);
		
		JsonObject command = compositeStep.getCommand();		
			
		//Adjust the move with the Pen module it's height.
		command = adjustMoveWithDimensions(command, new Vector3(0, 0, PEN_SIZE));
		command.addProperty(ModuleActor.MAX_ACCELERATION, MAX_ACCELERATION);
		command.addProperty(FORCE_STRAIGHT_LINE, false);
			
		//Translate it's parents composite steps into hardware steps.
		compositeStep = new CompositeStep(compositeStep.getProductStep(), command, compositeStep.getRelativeTo());	
		ArrayList<HardwareStep> hStep = forwardCompositeStep(compositeStep);
		if (hStep != null){
			translatedHardwareSteps.addAll(hStep);
		}
		
		return translatedHardwareSteps;
	}
}