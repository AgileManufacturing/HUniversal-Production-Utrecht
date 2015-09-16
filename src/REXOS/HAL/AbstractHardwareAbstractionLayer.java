package HAL;

import generic.Mast;
import generic.Mast.Mode;
import generic.Mast.State;

import java.text.ParseException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.json.JSONException;
import org.json.JSONObject;

import util.configuration.Configuration;
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
import HAL.listeners.EquipletCommandListener;
import HAL.listeners.EquipletListener;
import HAL.listeners.ExecutionProcessListener;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;
import HAL.listeners.TranslationProcessListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;
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
public abstract class AbstractHardwareAbstractionLayer implements ModuleListener, EquipletListener, TranslationProcessListener {
	public static final String ROS_INTERFACE_IMPLEMENTATION_PATH = "rosInterface/usedImplementation/";
	public static final String BLACKBOARD_IMPLEMENTATION = "blackboard";
	public static final String BRIDGE_IMPLEMENTATION = "rosBridge";
	public static final String JAVA_NODE_IMPLEMENTATION = "javaNode";
	
	protected CapabilityFactory capabilityFactory;
	protected ModuleFactory moduleFactory;
	protected ReconfigHandler reconfigHandler;
	protected RosInterface rosInterface;
	
	protected Map<ProductStep, TranslationProcess> translationProcesses;

	protected String equipletName;
	protected boolean isShadow;
	
	protected EquipletListener equipletListener;
	protected EquipletCommandListener equipletCommandListener;
	protected ModuleListener moduleListener;
	protected TranslationProcessListener translationProcessListener;
	protected ExecutionProcessListener executionProcessListener;

	public String getEquipletName() {
		return equipletName;
	}
	public boolean isShadow() {
		return isShadow;
	}

	public AbstractHardwareAbstractionLayer(String equipletName, boolean isShadow, HardwareAbstractionLayerListener hardwareAbstractionLayerListener) 
			throws KnowledgeException, BlackboardUpdateException {
		this(equipletName, isShadow, hardwareAbstractionLayerListener, hardwareAbstractionLayerListener, 
				hardwareAbstractionLayerListener, hardwareAbstractionLayerListener, hardwareAbstractionLayerListener);
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
	public AbstractHardwareAbstractionLayer(String equipletName, boolean isShadow, 
			EquipletListener equipletListener, ModuleListener moduleListener, TranslationProcessListener translationProcessListener,
			ExecutionProcessListener executionProcessListener, EquipletCommandListener equipletCommandListener) 
			throws KnowledgeException, BlackboardUpdateException {
		this.equipletName = equipletName;
		this.isShadow = isShadow;
		this.equipletListener = equipletListener;
		this.moduleListener = moduleListener;
		this.translationProcessListener = translationProcessListener;
		this.executionProcessListener = executionProcessListener;
		this.equipletCommandListener = equipletCommandListener;
		
		capabilityFactory = new CapabilityFactory(this);
		moduleFactory = new ModuleFactory(this, this);
		reconfigHandler = new ReconfigHandler(this, capabilityFactory, moduleFactory);
		
		// select correct ros interface implementation (this must match the one used by ROS)
		String usedImplementation = (String) Configuration.getProperty(ROS_INTERFACE_IMPLEMENTATION_PATH, equipletName);
		if(usedImplementation.equals(BLACKBOARD_IMPLEMENTATION)) this.rosInterface = new BlackboardRosInterface(this);
		else if(usedImplementation.equals(BRIDGE_IMPLEMENTATION)) this.rosInterface = new BridgeRosInterface(this);
		else if(usedImplementation.equals(JAVA_NODE_IMPLEMENTATION)) this.rosInterface = new NodeRosInterface(this);
		else throw new RuntimeException("rosInterface implementation is unknown");
		this.rosInterface.addEquipletListener(this);
		
		this.translationProcesses = new HashMap<ProductStep, TranslationProcess>();
	}
	
	/**
	 * This method will attempt to execute the set of hardware steps provided. This is a asynchronous call.
	 * Once the execution has finished, the onExecutionFinished of the HardwareAbstractionLayerListener will be called.
	 * 
	 * @param hardwareSteps
	 */
	protected void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps) {
		ExecutionProcess executionProcess = new ExecutionProcess(this.executionProcessListener, hardwareSteps, moduleFactory);
		executionProcess.start();
	}

	/**
	 * This method will attempt to translate the product step provided. This is a asynchronous call.
	 * Once the translation has finished, the onTranslationFinished of the HardwareAbstractionLayerListener will be called.
	 */
	protected void translateProductStep(String service, JSONObject criteria) {
		ProductStep step = new ProductStep(service, criteria);
		synchronized (translationProcesses) {
			TranslationProcess translationProcess = new TranslationProcess(this, step, capabilityFactory);
			translationProcess.start();
			translationProcesses.put(step, translationProcess);
		}
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
	protected boolean insertModule(JSONObject jsonStaticSettings, JSONObject jsonDynamicSettings) throws InvalidMastModeException {
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
	protected boolean updateModule(JSONObject jsonStaticSettings, JSONObject jsonDynamicSettings) throws InvalidMastModeException {
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
	protected JSONObject deleteModule(ModuleIdentifier moduleIdentifier) throws Exception {
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
	 * This method will return all the (most likely) supported capabilities from services
	 * @return ArrayList Capabilities
	 */
	public ArrayList<String> getSupportedCapabilitiesForService(String service){
		ArrayList<Capability> capabilities = new ArrayList<Capability>();
		ArrayList<String> supportedCapabilities = new ArrayList<String>();
		capabilities = capabilityFactory.getCapabilitiesForService(service);
		for (Capability capibility : capabilities) {
			String capabilityName = capibility.getName();
			supportedCapabilities.add(capabilityName);
		}
		return supportedCapabilities;
	}

	/**
	 * This method will be called by the blackboard handler when the state of the equiplet has changed. Do not call this method!
	 */
	@Override
	public void onModuleStateChanged(ModuleIdentifier module, Mast.State state) {
		moduleListener.onModuleStateChanged(module, state);
	}

	/**
	 * This method will be called by the blackboard handler when the mode of the equiplet has changed. Do not call this method!
	 */
	@Override
	public void onModuleModeChanged(ModuleIdentifier module, Mast.Mode mode) {
		moduleListener.onModuleModeChanged(module, mode);
	}

	/**
	 * Do not call this method!
	 */
	public ModuleFactory getModuleFactory() {
		return moduleFactory;
	}
	
	public ArrayList<Module> getModules() {
		return moduleFactory.getModules();
	}

	/**
	 * Do not call this method!
	 */
	public RosInterface getRosInterface() {
		return rosInterface;
	}

	@Override
	public void onEquipletStateChanged(Mast.State state) {
		equipletListener.onEquipletStateChanged(state);
	}
	@Override
	public void onEquipletModeChanged(Mast.Mode mode) {
		equipletListener.onEquipletModeChanged(mode);
	}
	@Override
	public void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		equipletListener.onEquipletCommandStatusChanged(status);
		
	}

	public void shutdown() {
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "HAL is shutting down");
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
	
	@Override
	public void onTranslationFinished(ProductStep step, ArrayList<HardwareStep> hardwareSteps) {
		synchronized (translationProcesses) {
			translationProcesses.remove(step);
		}
		translationProcessListener.onTranslationFinished(step, hardwareSteps);
	}

	@Override
	public void onTranslationFailed(ProductStep step) {
		synchronized (translationProcesses) {
			translationProcesses.remove(step);
		}
		translationProcessListener.onTranslationFailed(step);
	}
}
