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
import libraries.math.Matrix;
import libraries.math.RotationAngles;
import libraries.math.Vector3;
/**
 * Abstract representation of a actor module in HAL 
 * @author Bas Voskuijlen
 *
 */
public abstract class ModuleActor extends Module {
	protected static final String COMMAND = "command";
	protected static final String MODULE_COMMAND = "module_command";
	protected static final String DESTINATION = "destination";
	protected static final String LOOK_UP = "look_up";
	protected static final String NULL = "NULL";
	protected static final String X = "x";
	protected static final String Y = "y";
	protected static final String Z = "z";
	
	//TODO Check this (added by Rolf)
	protected static final String ROTATION_X = "rotationX";
	protected static final String ROTATION_Y = "rotationY";
	protected static final String ROTATION_Z = "rotationZ";
	
	protected static final String MOVE = "move";
	
	
	/**
	 * The blackboard client used for writing the hardware steps.
	 * The results from ROS are not processed with this client.
	 */
	protected BlackboardClient mongoClient;
	protected static final String MONGO_HOST = "145.89.191.131";
	
	/**
	 * Constructs a new ModuleActor and connects to the blackboard.
	 * @param moduleIdentifier
	 * @param moduleFactory
	 * @param moduleListener
	 * @throws KnowledgeException
	 * @throws UnknownHostException
	 * @throws GeneralMongoException
	 */
	public ModuleActor(ModuleIdentifier moduleIdentifier, ModuleFactory moduleFactory, ModuleListener moduleListener) 
			throws KnowledgeException, UnknownHostException, GeneralMongoException {
		super(moduleIdentifier, moduleFactory,moduleListener);
		mongoClient = new BlackboardClient(MONGO_HOST);
		try {
			mongoClient.setDatabase(moduleFactory.getHAL().getEquipletName());
			mongoClient.setCollection("EquipletStepsBlackBoard");
		} catch (InvalidDBNamespaceException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	public void setModuleListener(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	/**
	 * Executes a command by inserting it in the blackboard.
	 * @param command
	 * @throws ModuleExecutingException
	 */
	protected void executeMongoCommand(String command) throws ModuleExecutingException{
		try {
			mongoClient.insertDocument(command.toString());
		} catch (InvalidJSONException ex) {
			throw new ModuleExecutingException("Executing invalid JSON", ex);
		} catch (InvalidDBNamespaceException ex) {
			throw new ModuleExecutingException("Executing invalid DBNamespace", ex);
		} catch (GeneralMongoException ex) {
			throw new ModuleExecutingException("General mongo exception while trying to execute", ex);
		}
	}
	
	/**
	 * Forwards the remainder of the {@link CompositeStep} to the parent after this module has translated the CompositeStep
	 * @see http://wiki.agilemanufacturing.nl/index.php/HAL#Translation
	 * @param compositeStep
	 * @return The hardware steps resulted from the translation of the CompositeStep done by the parent modules.
	 * @throws ModuleTranslatingException if the CompositeStep could not completely be translated (which is the case if there is no parent module and the CompositeStep is not empty)  
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 */
	protected ArrayList<HardwareStep> forwardCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException{
		ModuleActor moduleActor = (ModuleActor) getParentModule();
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
	/**
	 * This method will execute the hardware step and forward any result to the {@link ProcessListener}
	 * @param processListener
	 * @param hardwareStep
	 * @throws ModuleExecutingException
	 */
	public void executeHardwareStep(ProcessListener processListener, HardwareStep hardwareStep) throws ModuleExecutingException{
		this.processListener = processListener;
		JsonObject command = hardwareStep.getRosCommand();
		executeMongoCommand(command.toString());
	}
	/**
	 * This method will translate the {@link CompositeStep} and forward the remainder to its parent.
	 * @param compositeStep
	 * @return The hardware steps resulted from the translation of the CompositeStep. 
	 * @throws ModuleTranslatingException
	 * @throws FactoryException
	 * @throws JarFileLoaderException
	 */
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JarFileLoaderException;
	/**
	 * This method will forward the changed MAST module state to the {@link ModuleListener}
	 * Do not call this method!
	 */
	public void onModuleStateChanged(String state){
		moduleListener.onModuleStateChanged(state, this);
	}
	/**
	 * This method will forward the changed MAST module mode to the {@link ModuleListener}
	 * Do not call this method!
	 */
	public void onModuleModeChanged(String mode){
		moduleListener.onModuleModeChanged(mode, this);
	}
	
	protected JsonObject adjustMoveWithDimensions(JsonObject compositeCommand, Vector3 offsetVector){
		return adjustMoveWithDimensions(compositeCommand, offsetVector, new RotationAngles(0, 0, 0));
	}
	protected JsonObject adjustMoveWithDimensions(JsonObject compositeCommand, Vector3 offsetVector, RotationAngles directionAngles){
		System.out.println("Adjusting move with dimentions: " + compositeCommand.toString() + 
				", offsetVector: " + offsetVector + " directionAngles: " + directionAngles);
		
		JsonObject originalMove = compositeCommand.remove(MOVE).getAsJsonObject();
		double originalX = originalMove.get(X).getAsDouble();
		double originalY = originalMove.get(Y).getAsDouble();
		double originalZ = originalMove.get(Z).getAsDouble();
		
		Matrix rotationMatrix = directionAngles.generateRotationMatrix();
		
		Vector3 originalVector = offsetVector;
		Vector3 rotatedVector = originalVector.rotate(rotationMatrix);
		
		JsonObject adjustedMove = new JsonObject();
		adjustedMove.addProperty(X, originalX + rotatedVector.x);
		adjustedMove.addProperty(Y, originalY + rotatedVector.y);
		adjustedMove.addProperty(Z, originalZ + rotatedVector.z);
		
		
		//TODO added by Rolf, Check this!!!!!!!!!!!!!!!!!!!
		adjustedMove.addProperty(ROTATION_X, originalMove.get(ROTATION_X).getAsDouble());
		adjustedMove.addProperty(ROTATION_Y, originalMove.get(ROTATION_Y).getAsDouble());
		adjustedMove.addProperty(ROTATION_Z, originalMove.get(ROTATION_Z).getAsDouble());
		
		compositeCommand.add(MOVE, adjustedMove);
		return compositeCommand;
	}
	


	@Override
	public void onProcessStatusChanged(String state) {
		// TODO Auto-generated method stub
		try {
			if(processListener != null){
				processListener.onProcessStateChanged(state, 0, this);
				if(state.equals("DONE")){
					processListener =null;
				}
			}
		} catch (HardwareAbstractionLayerProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
