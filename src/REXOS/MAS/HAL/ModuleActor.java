package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import HAL.exceptions.HardwareAbstractionLayerProcessException;
import HAL.exceptions.ModuleExecutingException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.listeners.ModuleListener;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.blackboard_client.data_classes.InvalidJSONException;
import libraries.knowledgedb_client.KeyNotFoundException;
import libraries.knowledgedb_client.KnowledgeException;

public abstract class ModuleActor extends Module {//implements mongolistener
	protected static final String COMMAND = "command";
	protected static final String MODULE_COMMAND = "module_command";
	protected static final String DESTINATION = "destination";
	protected static final String LOOK_UP = "look_up";
	protected static final String NULL = "NULL";
	protected static final String X = "x";
	protected static final String Y = "y";
	protected static final String Z = "z";
	protected static final String MOVE = "move";
	
	
	
	protected ModuleListener moduleListener;
	protected BlackboardClient mongoClient;
	protected static final String MONGO_HOST = "145.89.191.131";
	
	public ModuleActor(ModuleIdentifier moduleIdentifier) throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier);
		mongoClient = new BlackboardClient(MONGO_HOST);
	}
	
	public void setModuleListener(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	
	protected void executeMongoCommand(String command) throws ModuleExecutingException{
		try {
			mongoClient.insertDocument(command.toString());
		} catch (InvalidJSONException e) {
			throw new ModuleExecutingException("Executing invalid JSON",e);
		} catch (InvalidDBNamespaceException e) {
			throw new ModuleExecutingException("Executing invalid DBNamespace",e);
		} catch (GeneralMongoException e) {
			throw new ModuleExecutingException("General mongo exception while trying to execute",e);
		}
	}
	
	protected ArrayList<HardwareStep> forwardCompositeStep(CompositeStep compositeStep, JsonObject command) throws KnowledgeException, KeyNotFoundException, ModuleTranslatingException, Exception{
		ModuleActor moduleActor = null;
		moduleActor = (ModuleActor) getParentModule();
		if (moduleActor != null){
			return(moduleActor.translateCompositeStep(compositeStep));
		}
		else { //Root module, no more parents			
			//Check for remaining commands, then not capable
			JsonArray array = command.getAsJsonArray();
			if (array.size() > 0){
				throw new ModuleTranslatingException("The compositestep isn't completely empty.");
			}
			return null;
		}
	}
	
	abstract public void executeHardwareStep(HardwareStep hardwareStep) throws ModuleExecutingException;
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws KnowledgeException, KeyNotFoundException, ModuleTranslatingException, Exception;
	
	public void onHardwareStepChanged(String state, long hardwareStepSerialId) throws HardwareAbstractionLayerProcessException{
		moduleListener.onProcessStateChanged(state, hardwareStepSerialId, this);
	}
	public void onModuleStateChanged(String state){
		moduleListener.onModuleStateChanged(state, this);
	}
	public void onModuleModeChanged(String mode){
		moduleListener.onModuleModeChanged(mode, this);
	}
	
	protected JsonObject adjustMoveWithDimentions(JsonObject command, double height){
		JsonElement move = command.remove(MOVE);
		JsonArray moveParameters = move.getAsJsonArray();
		double x = moveParameters.get(0).getAsDouble();
		double y = moveParameters.get(1).getAsDouble();
		double z = moveParameters.get(2).getAsDouble();
		JsonObject mParameters = new JsonObject();
		mParameters.addProperty(X,x);
		mParameters.addProperty(Y,y-height);
		mParameters.addProperty(Z,z);
		moveParameters = new JsonArray();
		moveParameters.add(mParameters.get(X));
		moveParameters.add(mParameters.get(Y));
		moveParameters.add(mParameters.get(Z));
		command.add(MOVE, moveParameters);		
		return command;		
	}
}
