package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import com.google.gson.JsonElement;
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

public class Gripper extends ModuleActor {
	//Gonna be loaded from KDB:
	private static final double GRIPPER_SIZE = 18.24; // in cm
	
	private static final String ACTIVATE = "activate";
	private static final String DEACTIVATE = "deactivate";
	private static final String PICK = "pick";
	private static final String PLACE = "place";

	
	public Gripper(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener, ProcessListener processListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener, processListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject command = compositeStep.getCommand();
		command = adjustMoveWithDimentions(command, GRIPPER_SIZE);
		JsonElement pick = command.remove(PICK);
		JsonElement place = command.remove(PLACE);
		
		compositeStep = new CompositeStep(compositeStep.getProductStep(),command);		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(compositeStep);
		if (hStep != null)
			hardwareSteps.addAll(hStep);
		
		//Set hardwareSteps
		if (pick != null || place != null){
			JsonObject hardwareCommand = new JsonObject();
			if (pick != null){
				hardwareCommand.addProperty(COMMAND, ACTIVATE);
			}
			else{
				hardwareCommand.addProperty(COMMAND, DEACTIVATE);
			}
			
			hardwareSteps.add(new HardwareStep(compositeStep,hardwareCommand,moduleIdentifier));
		}
		
		return hardwareSteps;
	}
}
