package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import com.google.gson.JsonObject;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.dynamicloader.JarFileLoaderException;
import libraries.knowledgedb_client.KnowledgeException;
import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;

public class Pen extends ModuleActor {
	private static final double PEN_SIZE = 96.6; // in cm
	private static final int MAX_ACCELERATION = 50;

	public Pen(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener, ProcessListener processListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener, processListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject jsonCommand = compositeStep.getCommand();
		JsonObject command = jsonCommand.remove(COMMAND).getAsJsonObject();
		
		if (command != null){
			//Remove corresponding commands to module.
			if (command.remove("draw") == null){
				throw new ModuleTranslatingException ("Pen module didn't find a \"draw\" key in CompositeStep command: "+jsonCommand.toString());
			}			
			
			//Adjust the move with the Pen module it's height.
			command = adjustMoveWithDimentions(command, PEN_SIZE);
			command.addProperty("maxAcceleration", MAX_ACCELERATION);
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