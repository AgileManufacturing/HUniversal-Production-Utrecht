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
import HAL.exceptions.ModuleExecutingException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;

public class DeltaRobot extends ModuleActor {
	private static final String MOVE_DELTA_ROBOT = "move_delta_robot";

	public DeltaRobot(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory);
	}

	@Override
	public void executeHardwareStep(HardwareStep hardwareStep) throws ModuleExecutingException {
		JsonObject command = hardwareStep.getCommand();
		String moduleCommand = command.remove(MODULE_COMMAND).getAsString();
		if (moduleCommand.equals(MOVE_DELTA_ROBOT)){
			executeMongoCommand(command.toString());
		}
		else {
			//Impossible unless a retard calls this method
		}
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject command = compositeStep.getCommand();
		JsonObject move = command.remove(MOVE).getAsJsonObject();
		
		compositeStep = new CompositeStep(compositeStep.getProductStep(),command);		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(compositeStep,command);
		if (hStep != null)
			hardwareSteps.addAll(hStep);
		
		//Set hardwareSteps
		JsonObject hardwareCommand = new JsonObject();
		hardwareCommand.addProperty(MODULE_COMMAND, MOVE_DELTA_ROBOT);
		JsonObject m = new JsonObject();
		m.addProperty(X,move.remove(X).getAsNumber());
		m.addProperty(Y,move.remove(Y).getAsNumber());
		m.addProperty(Z,move.remove(Z).getAsNumber());
		m.addProperty("maxAcceleration",50);
		hardwareCommand.addProperty("command", "move");
		hardwareCommand.addProperty("destination","deltarobot" );
		hardwareCommand.addProperty("look_up","FIND_ID" );
		JsonObject parameters = new JsonObject();
		parameters.addProperty("ID", "Paper");
		hardwareCommand.add("look_up_parameters",parameters);
		hardwareCommand.add("payload",m);
		
		JsonObject jsonCommand = new JsonObject();
		jsonCommand.add("instructionData",hardwareCommand);
		jsonCommand.addProperty("moduleId",1);
		jsonCommand.addProperty("status","WAITING");
		jsonCommand.add("statusData",new JsonObject());
		
		jsonCommand.addProperty("nextStep","NULL");
		jsonCommand.addProperty("serviceStepID","NULL");
		
		JsonObject timeData = new JsonObject();
		timeData.addProperty("duration", 59);
		jsonCommand.add("timeData",timeData);
		
		
		hardwareSteps.add(new HardwareStep(compositeStep,hardwareCommand,moduleIdentifier));
		
		return hardwareSteps;
	}
}
