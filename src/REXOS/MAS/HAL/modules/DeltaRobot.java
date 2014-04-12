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

public class DeltaRobot extends ModuleActor {
	public final static int MAX_ACCELERATION = 50;
	
	public DeltaRobot(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject command = compositeStep.getCommand();
		JsonObject move = command.remove(MOVE).getAsJsonObject();
		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(new CompositeStep(compositeStep.getProductStep(),command));
		if (hStep != null)
			hardwareSteps.addAll(hStep);
		
		//Set hardwareSteps
		JsonObject hardwareCommand = new JsonObject();
		System.out.println(command.toString());
		int maxAcceleration = move.get("maxAcceleration").getAsInt();
		/*int subjectMaxAcceleration = compositeStep.getProductStep().getCriteria().get("partProperties").getAsJsonObject().get("maxAcceleration").getAsInt();
		if (subjectMaxAcceleration > maxAcceleration) maxAcceleration = subjectMaxAcceleration;*/
		if (maxAcceleration > MAX_ACCELERATION) maxAcceleration = MAX_ACCELERATION;
		
		hardwareCommand.addProperty(COMMAND, "move");
		hardwareCommand.addProperty("look_up","NULL" );
		//JsonObject parameters = new JsonObject();
		//parameters.addProperty("ID", "Paper");
		//hardwareCommand.add("look_up_parameters",parameters);
		hardwareCommand.add("payload",move);
		
		JsonObject jsonCommand = new JsonObject();
		jsonCommand.add("instructionData",hardwareCommand);
		jsonCommand.add("moduleIdentifier",moduleIdentifier.getAsJSON());
		jsonCommand.addProperty("status","WAITING");
		
		
		
		
		hardwareSteps.add(new HardwareStep(compositeStep,jsonCommand,moduleIdentifier));
		
		return hardwareSteps;
	}
}
