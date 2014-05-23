package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KnowledgeException;
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
	private static final double PEN_SIZE = 96.6; // in mm
	private static final int MAX_ACCELERATION = 50;

	public Pen(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject jsonCommand = compositeStep.getCommand();
		JsonObject command = jsonCommand.remove(COMMAND).getAsJsonObject();
		
		if (command != null){
			//Remove corresponding commands to module.
			if (command.remove("draw") == null){
				throw new ModuleTranslatingException ("Pen module didn't find a \"draw\" key in CompositeStep command: " + command.toString());
			}			
			
			//Adjust the move with the Pen module it's height.
			command = adjustMoveWithDimentions(command, PEN_SIZE);
			command.addProperty("maxAcceleration", MAX_ACCELERATION);
			command.addProperty("forceStraightLine", false);
			jsonCommand.add(COMMAND, command);
			
			//Translate it's parents composite steps into hardware steps.
			compositeStep = new CompositeStep(compositeStep.getProductStep(),jsonCommand);		
			ArrayList<HardwareStep> hStep = forwardCompositeStep(compositeStep);
			if (hStep != null)
				hardwareSteps.addAll(hStep);
		}
		else {
			throw new ModuleTranslatingException ("Pen module didn't receive any \"command\" key in CompositeStep command: "+jsonCommand.toString());
		}
		
		return hardwareSteps;
	}
}