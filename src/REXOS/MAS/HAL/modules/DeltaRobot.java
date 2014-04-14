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

public class DeltaRobot extends ModuleActor {
	public final static int MAX_ACCELERATION = 50;
	
	public DeltaRobot(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory, moduleListener);
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException {
		ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
		
		JsonObject jsonCommand = compositeStep.getCommand();
		JsonObject command = jsonCommand.get(COMMAND).getAsJsonObject();
		
		if (command != null){
			//Get move
			JsonObject move = command.remove(MOVE).getAsJsonObject();			
			if (move == null){
				throw new ModuleTranslatingException ("DeltaRobot module didn't find a \"move\" key in CompositeStep command: "+jsonCommand.toString());
			}
			
			
			
			//Set hardwareSteps
			JsonObject hardwareCommand = new JsonObject();
			
			//Adjust for maxAcceleration
			int maxAcceleration = MAX_ACCELERATION;
			if (move.get("maxAcceleration") != null)
				maxAcceleration = move.get("maxAcceleration").getAsInt();
			JsonElement partProperties = compositeStep.getProductStep().getCriteria().get("partProperties");
			if (partProperties != null){
				JsonElement subjectMaxAccelerationJson = partProperties.getAsJsonObject().get("maxAcceleration");
				if (subjectMaxAccelerationJson != null){
					if (subjectMaxAccelerationJson.getAsInt() < maxAcceleration){
						maxAcceleration = subjectMaxAccelerationJson.getAsInt();
					}
				}
			}
			if (maxAcceleration > MAX_ACCELERATION) maxAcceleration = MAX_ACCELERATION;
			
			
			hardwareCommand.addProperty(COMMAND, "move");
			
			//Add target to move relative to
			hardwareCommand.addProperty("look_up","FIND_ID" );
			JsonObject parameters = new JsonObject();
			parameters.addProperty("ID", jsonCommand.get("look_up").getAsString());
			hardwareCommand.add("look_up_parameters",parameters);
			
			
			JsonObject hardwareJsonCommand = new JsonObject();
			hardwareJsonCommand.add("moduleIdentifier",moduleIdentifier.getAsJSON());
			hardwareJsonCommand.addProperty("status","WAITING");
			
			//Add hopping a.k.a. safe movement pane
			if (command.get("forceStraightLine") != null){
				if (!command.get("forceStraightLine").getAsBoolean()){
					//Entry point
					int z = move.remove(Z).getAsInt();
					z -= 20; //20cm above actual point
					move.addProperty(Z, z);
					hardwareCommand.add("payload",move);
					hardwareJsonCommand.add("instructionData",hardwareCommand);
					hardwareSteps.add(new HardwareStep(compositeStep,hardwareJsonCommand,moduleIdentifier));
					
					//Actual point
					z = move.remove(Z).getAsInt();
					z += 20; //actual point
					move.addProperty(Z, z);
					hardwareCommand.remove("payload");
					hardwareCommand.add("payload",move);
					hardwareJsonCommand.remove("instructionData");
					hardwareJsonCommand.add("instructionData",hardwareCommand);
					hardwareSteps.add(new HardwareStep(compositeStep,hardwareJsonCommand,moduleIdentifier));
					
					//Exit point
					z = move.remove(Z).getAsInt();
					z -= 20; //20cm above actual point
					move.addProperty(Z, z);
					hardwareCommand.remove("payload");
					hardwareCommand.add("payload",move);
					hardwareJsonCommand.remove("instructionData");
					hardwareJsonCommand.add("instructionData",hardwareCommand);
					hardwareSteps.add(new HardwareStep(compositeStep,hardwareJsonCommand,moduleIdentifier));
				}
			}
			else {
				//Straight line
				hardwareCommand.add("payload",move);
				hardwareJsonCommand.add("instructionData",hardwareCommand);
				hardwareSteps.add(new HardwareStep(compositeStep,hardwareJsonCommand,moduleIdentifier));				
			}
			

			ArrayList<HardwareStep> hStep = forwardCompositeStep(new CompositeStep(compositeStep.getProductStep(),jsonCommand));
			if (hStep != null)
				hardwareSteps.addAll(hStep);
		}
		else {
			throw new ModuleTranslatingException ("DeltaRobot module didn't receive any \"command\" key in CompositeStep command: "+jsonCommand.toString());
		}
		
		return hardwareSteps;
	}
}
