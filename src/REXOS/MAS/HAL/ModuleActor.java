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
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
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
	
	public void executeHardwareStep(ProcessListener processListener, HardwareStep hardwareStep) throws ModuleExecutingException{
		JsonObject command = hardwareStep.getCommand();
		executeMongoCommand(command.toString());
		this.processListener = processListener;
	}
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException;
	
	public void onHardwareStepChanged(String state, long hardwareStepSerialId) throws HardwareAbstractionLayerProcessException{
		processListener.onProcessStateChanged(state, hardwareStepSerialId, this);
	}
	public void onModuleStateChanged(String state){
		moduleListener.onModuleStateChanged(state, this);
	}
	public void onModuleModeChanged(String mode){
		moduleListener.onModuleModeChanged(mode, this);
	}
	
	protected JsonObject adjustMoveWithDimentions(JsonObject command, double height){
		System.out.println("Adjusting move with dimentions: "+command.toString());
		JsonObject move = command.remove(MOVE).getAsJsonObject();
		double x = move.get(X).getAsDouble();
		double y = move.get(Y).getAsDouble();
		double z = move.get(Z).getAsDouble();
		JsonObject mParameters = new JsonObject();
		mParameters.addProperty(X,x);
		mParameters.addProperty(Y,y);
		mParameters.addProperty(Z,z+height);
		command.add(MOVE, mParameters);		
		return command;		
	}
	


	@Override
	public void onProcessStateChanged(String state) {
		// TODO Auto-generated method stub
		try {
			processListener.onProcessStateChanged(state, 0, this);
		} catch (HardwareAbstractionLayerProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
