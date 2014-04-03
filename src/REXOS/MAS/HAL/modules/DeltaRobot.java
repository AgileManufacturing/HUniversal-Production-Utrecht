package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import com.google.gson.JsonObject;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.knowledgedb_client.KnowledgeException;
import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.exceptions.ModuleExecutingException;

public class DeltaRobot extends ModuleActor {
	private static final String MOVE_DELTA_ROBOT = "move_delta_robot";

	public DeltaRobot(ModuleIdentifier moduleIdentifier) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier);
	}

	@Override
	public void executeHardwareStep(HardwareStep hardwareStep) throws ModuleExecutingException {
		JsonObject command = hardwareStep.getCommand();
		String moduleCommand = command.get(MODULE_COMMAND).getAsString();
		if (moduleCommand.equals(MOVE_DELTA_ROBOT)){
			executeMongoCommand(command.toString());
		}
		else {
			//Impossible unless a retard calls this method
		}
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws Exception {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject command = compositeStep.getCommand();
		
		compositeStep = new CompositeStep(compositeStep.getProductStep(),command);		
		ArrayList<HardwareStep> hStep = forwardCompositeStep(compositeStep,command);
		if (hStep != null)
			hardwareSteps.addAll(hStep);
		
		//Set hardwareSteps
		JsonObject hardwareCommand = new JsonObject();
		hardwareCommand.addProperty(MODULE_COMMAND, MOVE_DELTA_ROBOT);
		hardwareSteps.add(new HardwareStep(compositeStep,hardwareCommand,moduleIdentifier));
		
		return hardwareSteps;
	}
}
