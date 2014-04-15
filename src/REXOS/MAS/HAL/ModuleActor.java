package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

import com.google.gson.JsonObject;

import HAL.exceptions.FactoryException;
import HAL.exceptions.HardwareAbstractionLayerProcessException;
import HAL.exceptions.ModuleExecutingException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;
import libraries.blackboard_client.BlackboardClient;
import libraries.blackboard_client.data_classes.GeneralMongoException;
import libraries.blackboard_client.data_classes.InvalidDBNamespaceException;
import libraries.blackboard_client.data_classes.InvalidJSONException;
import libraries.dynamicloader.JarFileLoaderException;
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
	
	
	
	protected BlackboardClient mongoClient;
	protected static final String MONGO_HOST = "145.89.191.131";
	
	public ModuleActor(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) 
			throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory,moduleListener);
		mongoClient = new BlackboardClient(MONGO_HOST);
		try {
			mongoClient.setDatabase("EQ1");
			mongoClient.setCollection("EquipletStepsBlackBoard");
		} catch (InvalidDBNamespaceException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	protected void executeMongoCommand(String command) throws ModuleExecutingException{
		System.out.println("Executing mongoCommand: "+command);
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
	
	protected ArrayList<HardwareStep> forwardCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException{
		ModuleActor moduleActor = null;
		moduleActor = (ModuleActor) getParentModule();
		if (moduleActor != null){
			return(moduleActor.translateCompositeStep(compositeStep));
		}
		else { //Root module, no more parents			
			//Check for remaining commands, then not capable
			if (compositeStep.getCommand().get(COMMAND).getAsJsonObject() == null){
				throw new ModuleTranslatingException("The compositestep doesn't contain any \"command\" key.");
			}
			if (!compositeStep.getCommand().get(COMMAND).getAsJsonObject().toString().trim().equalsIgnoreCase("{}")){
				throw new ModuleTranslatingException("The compositestep isn't completely empty." + 
						compositeStep.getCommand().get(COMMAND).getAsJsonObject());
			}
			return null;
		}
	}
	
	public synchronized void executeHardwareStep(ProcessListener processListener, HardwareStep hardwareStep) throws ModuleExecutingException{
		this.processListener = processListener;
		JsonObject command = hardwareStep.getCommand();
		executeMongoCommand(command.toString());
	}
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException;
	
	protected JsonObject adjustMoveWithDimentions(JsonObject command, double height){
		System.out.println("Adjusting move with dimentions: "+command.toString());
		JsonObject move = command.remove(MOVE).getAsJsonObject();
		double x = move.remove(X).getAsDouble();
		double y = move.remove(Y).getAsDouble();
		double z = move.remove(Z).getAsDouble();
		
		move.addProperty(X,x);
		move.addProperty(Y,y);
		move.addProperty(Z,z+height);
		command.add(MOVE, move);		
		return command;		
	}
	


	@Override
	public synchronized void onProcessStatusChanged(String status) {
		System.out.println("Process Status changed - ModuleActor");
		// TODO Auto-generated method stub
		try {
			if (processListener != null){
				System.out.println("Process Listener active - ModuleActor");
				processListener.onProcessStatusChanged(status, 0, this);
				processListener = null;
			}
		} catch (HardwareAbstractionLayerProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
