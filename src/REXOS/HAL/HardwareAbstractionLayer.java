package HAL;

import generic.Mast;
import generic.Mast.Mode;
import generic.Mast.State;

import java.text.ParseException;
import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.dataTypes.DynamicSettings;
import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.StaticSettings;
import HAL.exceptions.BlackboardUpdateException;
import HAL.exceptions.FactoryException;
import HAL.exceptions.InvalidMastModeException;
import HAL.factories.CapabilityFactory;
import HAL.factories.ModuleFactory;
import HAL.factories.ReconfigHandler;
import HAL.libraries.dynamicloader.JarFileLoaderException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.EquipletListener;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;
import HAL.steps.HardwareStep;
import HAL.tasks.ExecutionProcess;
import HAL.tasks.TranslationProcess;

/**
 * The main interface of the HAL for the equiplet agent. This class manages the factories and the blackboard handler.
 * 
 * @author Bas Voskuijlen
 * @author Lars Veenendaal
 * 
 * @see http://wiki.agilemanufacturing.nl/index.php/HAL
 */
public class HardwareAbstractionLayer implements ModuleListener, EquipletListener {
	private CapabilityFactory capabilityFactory;
	private ModuleFactory moduleFactory;
	private ReconfigHandler reconfigHandler;
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private RosInterface rosInterface;

	private String equipletName;

	public String getEquipletName() {
		return equipletName;
	}

	/**
	 * The constructor which will create a new HardwareAbstractionLayer. The factories and the blackboard handler will be constructed.
	 * A HAL should only be constructed once for every equiplet (and thus should only be constructed once by the equiplet agent).
	 * Please make sure that the HardwareAbstractionLayerListener has already set the equipletName, as this will be copied by the constructor.
	 * 
	 * @param hardwareAbstractionLayerListener
	 * @throws KnowledgeException
	 * @throws BlackboardUpdateException
	 */
	public HardwareAbstractionLayer(String equipletName, HardwareAbstractionLayerListener hardwareAbstractionLayerListener) 
			throws KnowledgeException, BlackboardUpdateException {
		this.equipletName = equipletName;

		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		capabilityFactory = new CapabilityFactory(this);
		moduleFactory = new ModuleFactory(this, this);
		reconfigHandler = new ReconfigHandler(this, capabilityFactory, moduleFactory);
		rosInterface = new NodeRosInterface(this);
		//rosInterface = new BlackboarRosInterface(this);
		//rosInterface = new BridgeRosInterface(this);
		rosInterface.addEquipletListener(this);
	}

	/**
	 * This method will attempt to execute the set of hardware steps provided. This is a asynchronous call.
	 * Once the execution has finished, the onExecutionFinished of the HardwareAbstractionLayerListener will be called.
	 * 
	 * @param hardwareSteps
	 */
	public void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps) {
		ExecutionProcess executionProcess = new ExecutionProcess(this.hardwareAbstractionLayerListener, hardwareSteps, moduleFactory);
		executionProcess.start();
	}

	/**
	 * This method will attempt to translate the product step provided. This is a asynchronous call.
	 * Once the translation has finished, the onTranslationFinished of the HardwareAbstractionLayerListener will be called.
	 */
	public void translateProductStep(String service, JSONObject criteria) {
		TranslationProcess translationProcess = new TranslationProcess(this.hardwareAbstractionLayerListener, service, criteria, capabilityFactory);
		translationProcess.start();
	}

	/**
	 * This method will insert a module by copying data from the staticSettings and dynamicSettings JSONObjects to the database and invoking the Equiplet Node to load the new
	 * modules.
	 * Make sure the equiplet is in Service Mode before calling this method.
	 * 
	 * @param staticSettings
	 * @param dynamicSettings
	 * @return true if insertion is successful.
	 * @throws InvalidMastModeException
	 *             if the equiplet is not in the correct mode.
	 */
	public boolean insertModule(JSONObject jsonStaticSettings, JSONObject jsonDynamicSettings) throws InvalidMastModeException {
		try {
			StaticSettings staticSettings = StaticSettings.deSerialize(jsonStaticSettings);
			DynamicSettings dynamicSettings = DynamicSettings.deSerialize(jsonDynamicSettings);
			return reconfigHandler.insertModule(staticSettings, dynamicSettings);
		} catch (JSONException | ParseException ex) {
			Logger.log(LogSection.HAL_RECONFIG, LogLevel.ERROR, "Parsing static of dynamic settings failed.", ex);
			return false;
		}
	}

	/**
	 * This method will update a module by copying data from the staticSettings and dynamicSettings JSONObjects to the database and invoking the Equiplet Node to reload the
	 * modules.
	 * The software of the module or the associated capabilities will be updated if the buildnumber is higher than the buildnumber of the already installed software.
	 * Make sure the equiplet is in Service Mode before calling this method.
	 * 
	 * @param staticSettings
	 * @param dynamicSettings
	 * @return
	 * @throws InvalidMastModeException
	 *             if the equiplet is not in the correct mode.
	 */
	public boolean updateModule(JSONObject jsonStaticSettings, JSONObject jsonDynamicSettings) throws InvalidMastModeException {
		try {
			StaticSettings staticSettings = StaticSettings.deSerialize(jsonStaticSettings);
			DynamicSettings dynamicSettings = DynamicSettings.deSerialize(jsonDynamicSettings);
			return reconfigHandler.updateModule(staticSettings, dynamicSettings);
		} catch (JSONException | ParseException ex) {
			Logger.log(LogSection.HAL_RECONFIG, LogLevel.ERROR, "Parsing static of dynamic settings failed.", ex);
			return false;
		}
		
	}

	/**
	 * This method will remove a module by moving data from the database to the JSONObject and invoking the Equiplet Node to remove the module.
	 * Make sure the equiplet is in Service Mode before calling this method.
	 * 
	 * @param moduleIdentifier
	 * @return The static settings of the module from the database.
	 * @throws InvalidMastModeException
	 *             if the equiplet is not in the correct mode.
	 * @throws JarFileLoaderException
	 *             if the loading of the jarFile by the factories failed
	 * @throws FactoryException
	 *             if the instantiation of the class in the jarFile failed
	 */
	public JSONObject deleteModule(ModuleIdentifier moduleIdentifier) throws Exception {
		StaticSettings staticSettings = reconfigHandler.removeModule(moduleIdentifier);
		return staticSettings.serialize();
	}

	/**
	 * This method will return the modules which have no child (and thus are the bottom modules)
	 * 
	 * @return
	 * @throws FactoryException
	 *             if the instantiation of the class in the jarFile failed
	 */
	public ArrayList<Module> getBottomModules() throws FactoryException {
		return moduleFactory.getBottomModules();
	}

	/**
	 * This method will return all the (most likely) supported services
	 * 
	 * @return
	 */
	public ArrayList<String> getSupportedServices() {
		return reconfigHandler.getAllSupportedServices();
	}

	/**
	 * This method will be called by the blackboard handler when the state of the equiplet has changed. Do not call this method!
	 */
	@Override
	public void onModuleStateChanged(Module module, Mast.State state) {
		hardwareAbstractionLayerListener.onModuleStateChanged(module, state);
	}

	/**
	 * This method will be called by the blackboard handler when the mode of the equiplet has changed. Do not call this method!
	 */
	@Override
	public void onModuleModeChanged(Module module, Mast.Mode mode) {
		hardwareAbstractionLayerListener.onModuleModeChanged(module, mode);
	}

	/**
	 * Do not call this method!
	 */
	public ModuleFactory getModuleFactory() {
		return moduleFactory;
	}

	/**
	 * Do not call this method!
	 */
	public RosInterface getRosInterface() {
		return rosInterface;
	}

	@Override
	public void onEquipletStateChanged(Mast.State state) {
		hardwareAbstractionLayerListener.onEquipletStateChanged(state);
	}
	@Override
	public void onEquipletModeChanged(Mast.Mode mode) {
		hardwareAbstractionLayerListener.onEquipletModeChanged(mode);
	}
	@Override
	public void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		hardwareAbstractionLayerListener.onEquipletCommandStatusChanged(status);
		
	}

	public void shutdown() {
		rosInterface.shutdown();
	}
	
	@Override
	public void finalize() throws Throwable {
		super.finalize();
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "HAL has been garbage collected");
	}

	public void changeState(State state) {
		try{
			JSONObject equipletCommand = new JSONObject();
			equipletCommand.put("command", "changeState");
			equipletCommand.put("status", "WAITING");
			JSONObject parameters = new JSONObject();
			parameters.put("desiredState", state.toString());
			equipletCommand.put("parameters", parameters);
			rosInterface.postEquipletCommand(equipletCommand);
		} catch(JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occured which is considered to be impossible", ex);
		}
	}

	public void changeMode(Mode mode) {
		try{
			JSONObject equipletCommand = new JSONObject();
			equipletCommand.put("command", "changeMode");
			equipletCommand.put("status", "WAITING");
			JSONObject parameters = new JSONObject();
			parameters.put("desiredMode", mode.toString());
			equipletCommand.put("parameters", parameters);
			rosInterface.postEquipletCommand(equipletCommand);
		} catch(JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occured which is considered to be impossible", ex);
		}
	}
	public void reloadRos() {
		try{
			JSONObject equipletCommand = new JSONObject();
			equipletCommand.put("command", "reload");
			equipletCommand.put("status", "WAITING");
			JSONObject parameters = new JSONObject();
			parameters.put("reload", "ALL_MODULES");
			equipletCommand.put("parameters", parameters);
			rosInterface.postEquipletCommand(equipletCommand);
		} catch(JSONException ex) {
			Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occured which is considered to be impossible", ex);
		}
	}
}
