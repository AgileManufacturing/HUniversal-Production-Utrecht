package HAL;

import java.net.UnknownHostException;
import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import util.math.Matrix;
import util.math.RotationAngles;
import util.math.Vector3;
import HAL.dataTypes.ModuleIdentifier;
import HAL.exceptions.FactoryException;
import HAL.exceptions.ModuleExecutingException;
import HAL.exceptions.ModuleTranslatingException;
import HAL.factories.ModuleFactory;
import HAL.libraries.blackboard_client.data_classes.GeneralMongoException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;
import HAL.steps.CompositeStep;
import HAL.steps.HardwareStep;
/**
 * Abstract representation of a actor module in HAL 
 * @author Bas Voskuijlen
 * @author Lars Veenendaal
 * 
 */
public abstract class ModuleActor extends Module implements ProcessListener {
	protected static final String MODULE_COMMAND = "module_command";
	protected static final String APPROACH = "approach";
	protected static final String DESTINATION = "destination";
	protected static final String NULL = "NULL";
	protected static final String MAX_ACCELERATION = "maxAcceleration";
	protected static final String FORCE_STRAIGHT_LINE = "forceStraightLine";
	protected static final String MOVE_X = "x";
	protected static final String MOVE_Y = "y";
	protected static final String MOVE_Z = "z";
	protected static final String MOVE_MAX_ACCELERATION = "maxAcceleration";
	
	protected static final String ROTATION_X = "rotationX";
	protected static final String ROTATION_Y = "rotationY";
	protected static final String ROTATION_Z = "rotationZ";
	
	protected static final String MOVE = "move";
	protected static final String ROTATE = "rotate";
	
	protected ArrayList<ProcessListener> processListeners;
	
	
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
			throws KnowledgeException {
		super(moduleIdentifier, moduleFactory,moduleListener);
		processListeners = new ArrayList<ProcessListener>();
		moduleFactory.getHAL().getRosInterface().addProcessListener(this);
	}
	public void setModuleListener(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	
	/**
	 * Forwards the remainder of the {@link CompositeStep} to the parent after this module has translated the CompositeStep
	 * @see http://wiki.agilemanufacturing.nl/index.php/HAL#Translation
	 * @param compositeStep
	 * @return The hardware steps resulted from the translation of the CompositeStep done by the parent modules.
	 * @throws ModuleTranslatingException if the CompositeStep could not completely be translated (which is the case if there is no parent module and the CompositeStep is not empty)  
	 * @throws FactoryException
	 * @throws JSONException 
	 */
	protected ArrayList<HardwareStep> forwardCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JSONException {
		ModuleActor moduleActor = (ModuleActor) getParentModule();
		if (moduleActor != null) {
			// forward to parent
			ArrayList<HardwareStep> hardwareSteps = moduleActor.translateCompositeStep(compositeStep);
			if (hardwareSteps != null){
				return hardwareSteps;
			}
			return new ArrayList<HardwareStep>();
		} else {
			// root module, no more parents			
			// if commands remain then the modules were not able to fully translate the compositeStep
			if (compositeStep.getCommand().keys().hasNext() == true){
				throw new ModuleTranslatingException("The compositestep isn't completely empty: \n" + 
						compositeStep.getCommand(), compositeStep);
			} else {
				Logger.log(LogSection.HAL_MODULES, LogLevel.DEBUG, "Root of the module tree has been reached, composite step succesfully translated");
				return new ArrayList<HardwareStep>();
			}
		}
	}
	/**
	 * This method will execute the hardware step and forward any result to the {@link ProcessListener}
	 * @param processListener
	 * @param hardwareStep
	 * @throws ModuleExecutingException
	 */
	public void executeHardwareStep(HardwareStep hardwareStep) {
		moduleFactory.getHAL().getRosInterface().postHardwareStep(hardwareStep);
	}
	/**
	 * This method will translate the {@link CompositeStep} and forward the remainder to its parent.
	 * @param compositeStep
	 * @return The hardware steps resulted from the translation of the CompositeStep. 
	 * @throws ModuleTranslatingException
	 * @throws FactoryException
	 * @throws JSONException 
	 */
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) throws ModuleTranslatingException, FactoryException, JSONException;
	
	/**
	 * Returns -1 if not found.
	 * 
	 */
	protected int getPlaceholderIndex(ArrayList<HardwareStep> hardwareSteps){
		if (hardwareSteps != null){
			for (int i = 0; i < hardwareSteps.size(); i++){
				if (hardwareSteps.get(i) == null){
					return i;
				}
			}
		}
		return -1;
	}
	
	protected CompositeStep adjustMoveWithDimensions(CompositeStep compositeStep, Vector3 offsetVector) throws JSONException{
		JSONObject command = compositeStep.getCommand();
		command = adjustMoveWithDimensions(command, offsetVector);
		return new CompositeStep(compositeStep.getService(), command, compositeStep.getOriginPlacement());
	}
	protected JSONObject adjustMoveWithDimensions(JSONObject compositeCommand, Vector3 offsetVector) throws JSONException{
		return adjustMoveWithDimensions(compositeCommand, offsetVector, new RotationAngles(0, 0, 0));
	}
	protected JSONObject adjustMoveWithDimensions(JSONObject compositeCommand, Vector3 offsetVector, RotationAngles directionAngles) throws JSONException{
		Logger.log(LogSection.HAL_MODULES, LogLevel.DEBUG, "Adjusting move with dimentions: " + compositeCommand.toString() + 
				", offsetVector: " + offsetVector + " directionAngles: " + directionAngles);
		
		JSONObject move = compositeCommand.getJSONObject(MOVE);
		if (move != null){
			double originalX = move.getDouble(MOVE_X);
			move.remove(MOVE_X);
			double originalY = move.getDouble(MOVE_Y);
			move.remove(MOVE_Y);
			double originalZ = move.getDouble(MOVE_Z);
			move.remove(MOVE_Z);
			
			Matrix rotationMatrix = directionAngles.generateRotationMatrix();
			
			Vector3 originalVector = offsetVector;
			Vector3 rotatedVector = originalVector.rotate(rotationMatrix);
			
			move.put(MOVE_X, originalX + rotatedVector.x);
			move.put(MOVE_Y, originalY + rotatedVector.y);
			move.put(MOVE_Z, originalZ + rotatedVector.z);
		}
		else {
			Logger.log(LogSection.HAL_TRANSLATION, LogLevel.NOTIFICATION, 
					"CompositeStep command does not contain any move key to adjust. " + compositeCommand);
		}
		return compositeCommand;
	}
	
	public void addProcessListener(ProcessListener processListener) {
		synchronized (processListeners) {
			processListeners.add(processListener);
		}
	}
	public void removeProcessListener(ProcessListener processListener) {
		synchronized (processListeners) {
			processListeners.remove(processListener);
		}
	}
	@Override
	public void onProcessStatusChanged(HardwareStep hardwareStep) {
		synchronized (processListeners) {
			if(hardwareStep.getModuleIdentifier() == this.moduleIdentifier) {
				for (ProcessListener processListener : processListeners) {
					processListener.onProcessStatusChanged(hardwareStep);
				}
			}
		}
	}
}
