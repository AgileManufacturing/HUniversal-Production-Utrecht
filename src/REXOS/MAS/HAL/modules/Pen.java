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

public class Pen extends ModuleActor {
	//Gonna be loaded from KDB:
	private static final double PEN_SIZE = 96.6; // in cm

	public Pen(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject command = compositeStep.getCommand();
		command = adjustMoveWithDimentions(command, PEN_SIZE);
		command.addProperty("forceStraightLine", true);
		
		compositeStep = new CompositeStep(compositeStep.getProductStep(),command);		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(compositeStep);
		if (hStep != null)
			hardwareSteps.addAll(hStep);
		
		return hardwareSteps;
	}
}