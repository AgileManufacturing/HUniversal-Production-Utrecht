package HAL.modules;

import java.net.UnknownHostException;
import java.util.ArrayList;

import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.knowledgedb_client.KnowledgeException;
import HAL.ModuleActor;
import HAL.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public class DeltaRobot extends ModuleActor {
	public final static int MAX_ACCELERATION = 50;
	
	public DeltaRobot(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject jsonCommand = compositeStep.getCommand();
		JsonObject command = jsonCommand.remove(HardwareStep.COMMAND).getAsJsonObject();
		
		if (command != null){
			//Get move
			if (command.has(MOVE) == false){
				throw new IllegalArgumentException("DeltaRobot module didn't find a \"move\" key in CompositeStep command: " + command.toString());
			}
			JsonObject move = command.remove(MOVE).getAsJsonObject();			
			
			
			
			//Set hardwareSteps
			JsonObject hardwareCommand = new JsonObject();
			
			//Adjust for maxAcceleration
			int maxAcceleration = MAX_ACCELERATION;
			if (move.get(ModuleActor.MAX_ACCELERATION) != null){
				if (move.get(ModuleActor.MAX_ACCELERATION).getAsInt() < maxAcceleration){
					maxAcceleration = move.remove(ModuleActor.MAX_ACCELERATION).getAsInt();
				}
			}
			JsonElement partProperties = compositeStep.getProductStep().getCriteria().get("partProperties");
			if (partProperties != null){
				JsonElement subjectMaxAccelerationJson = partProperties.getAsJsonObject().get(ModuleActor.MAX_ACCELERATION);
				if (subjectMaxAccelerationJson != null){
					if (subjectMaxAccelerationJson.getAsInt() < maxAcceleration){
						maxAcceleration = subjectMaxAccelerationJson.getAsInt();
					}
				}
			}
			if (maxAcceleration > MAX_ACCELERATION) maxAcceleration = MAX_ACCELERATION;
			move.addProperty(ModuleActor.MAX_ACCELERATION, maxAcceleration);
			
			hardwareCommand.addProperty(HardwareStep.COMMAND, "move");
			
			//Add target to move relative to
			hardwareCommand.addProperty("look_up","FIND_ID" );
			JsonObject parameters = new JsonObject();
			parameters.addProperty("ID", jsonCommand.get("look_up").getAsString());
			hardwareCommand.add("look_up_parameters",parameters);
			
			
			JsonObject hardwareJsonCommand = new JsonObject();
			hardwareJsonCommand.add("moduleIdentifier",moduleIdentifier.getAsJSON());
			hardwareJsonCommand.addProperty("status","WAITING");
			System.out.println("DeltaRobot: translating..");
			//Add hopping a.k.a. safe movement pane
			if (command.get("forceStraightLine") != null){
				System.out.println("DeltaRobot: forceStraightLine not null");
				if (!command.remove("forceStraightLine").getAsBoolean()){
					System.out.println("DeltaRobot: forceStraightLine true");
					JsonObject entry_move = new JsonParser().parse(move.toString()).getAsJsonObject();
					JsonObject actual_move = new JsonParser().parse(move.toString()).getAsJsonObject();
					JsonObject exit_move = new JsonParser().parse(move.toString()).getAsJsonObject();
					
					JsonObject entry_hardwareCommand = new JsonParser().parse(hardwareCommand.toString()).getAsJsonObject();
					JsonObject actual_hardwareCommand = new JsonParser().parse(hardwareCommand.toString()).getAsJsonObject();
					JsonObject exit_hardwareCommand = new JsonParser().parse(hardwareCommand.toString()).getAsJsonObject();

					JsonObject entry_point = new JsonParser().parse(hardwareJsonCommand.toString()).getAsJsonObject();
					JsonObject actual_point = new JsonParser().parse(hardwareJsonCommand.toString()).getAsJsonObject();
					JsonObject exit_point = new JsonParser().parse(hardwareJsonCommand.toString()).getAsJsonObject();

					System.out.println("DeltaRobot: points parsed");
					
					entry_hardwareCommand.add("payload",entry_move);
					entry_point.add("instructionData",entry_hardwareCommand);
					hardwareSteps.add(new HardwareStep(compositeStep,entry_point,moduleIdentifier));

					System.out.println("DeltaRobot: entry point added");
					//Actual point
					hardwareCommand.remove("payload");
					actual_hardwareCommand.add("payload",actual_move);
					actual_point.add("instructionData",actual_hardwareCommand);
					hardwareSteps.add(new HardwareStep(compositeStep,actual_point,moduleIdentifier));
					
					hardwareSteps.add(null);

					System.out.println("DeltaRobot: actual point added");
					//Exit point
					exit_hardwareCommand.add("payload",exit_move);
					exit_point.add("instructionData",exit_hardwareCommand);
					hardwareSteps.add(new HardwareStep(compositeStep,exit_point,moduleIdentifier));
					System.out.println("DeltaRobot: exit point added");
				}
				else {
					//Straight line
					hardwareCommand.add("payload",move);
					hardwareJsonCommand.add("instructionData",hardwareCommand);
					hardwareSteps.add(new HardwareStep(compositeStep,hardwareJsonCommand,moduleIdentifier));	
					System.out.println("DeltaRobot: straight line added");				
				}
			}
			else {
				//Straight line
				hardwareCommand.add("payload",move);
				hardwareJsonCommand.add("instructionData",hardwareCommand);
				hardwareSteps.add(new HardwareStep(compositeStep,hardwareJsonCommand,moduleIdentifier));		
				System.out.println("DeltaRobot: straight line added");							
			}
			
			jsonCommand.add(HardwareStep.COMMAND, command);
			ArrayList<HardwareStep> hStep = forwardCompositeStep(new CompositeStep(compositeStep.getProductStep(),jsonCommand));
			if (hStep != null) hardwareSteps.addAll(hStep);
		}
		else {
			throw new IllegalArgumentException ("DeltaRobot module didn't receive any \"command\" key in CompositeStep command" + compositeStep);
		}
		
		return hardwareSteps;
	}
}
