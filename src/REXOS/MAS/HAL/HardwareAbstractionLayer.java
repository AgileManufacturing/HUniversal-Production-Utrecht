package HAL;

import java.util.ArrayList;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import libraries.knowledgedb_client.KnowledgeException;
import HAL.capabilities.Capability;
import HAL.exceptions.BlackboardUpdateException;
import HAL.factories.CapabilityFactory;
import HAL.factories.ModuleFactory;
import HAL.listeners.BlackboardListener;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;
import HAL.tasks.ExecutionProcess;
import HAL.tasks.TranslationProcess;

public class HardwareAbstractionLayer implements ModuleListener{
	private CapabilityFactory capabilityFactory;
	private ModuleFactory moduleFactory;
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private BlackboardHandler blackboardHandler;
	
	// TODO move somewhere else?? 
	private String equipletName;
	public String getEquipletName() {
		return equipletName;
	}

	public HardwareAbstractionLayer(HardwareAbstractionLayerListener hardwareAbstractionLayerListener) throws KnowledgeException, BlackboardUpdateException{
		equipletName = hardwareAbstractionLayerListener.getEquipletName();
		
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		capabilityFactory = new CapabilityFactory(this);
		moduleFactory = new ModuleFactory(this, this);
		blackboardHandler = new BlackboardHandler(equipletName);
		
	}
	
	public void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps){
		ExecutionProcess executionProcess = new ExecutionProcess(this.hardwareAbstractionLayerListener, hardwareSteps, moduleFactory);
		executionProcess.start();
	}
	public void translateProductStep(ProductStep productStep){
		TranslationProcess translationProcess = new TranslationProcess(this.hardwareAbstractionLayerListener, productStep, capabilityFactory);
		translationProcess.start();
	}
	public ArrayList<Capability> getAllCapabilities() throws Exception{
		return capabilityFactory.getAllSupportedCapabilities();
	}
	
	public boolean insertModule(JsonObject staticSettings, JsonObject dynamicSettings){
		boolean isModuleAdditionSuccesful = moduleFactory.insertModule(staticSettings, dynamicSettings);
		JsonArray capabilities = staticSettings.get("type").getAsJsonObject().get("capabilities").getAsJsonArray();
		boolean isCapabilityAdditionSuccesful = capabilityFactory.insertCapabilities(capabilities);
		return isModuleAdditionSuccesful == true && isCapabilityAdditionSuccesful == true;
	}
	public boolean updateModule(JsonObject staticSettings, JsonObject dynamicSettings){
		return moduleFactory.updateModule(staticSettings, dynamicSettings);
	}
	public JsonObject deleteModule(ModuleIdentifier moduleIdentifier){
		try {
			JsonArray capabilities =capabilityFactory.removeCapabilities(moduleIdentifier);
			JsonObject module = moduleFactory.deleteModule(moduleIdentifier);			
			module.get("type").getAsJsonObject().add("capabilities", capabilities);
			return module;
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return null;
		}
	}
	public ArrayList<Module> getBottomModules() throws Exception{
		return moduleFactory.getBottomModules();
	}

	@Override
	public void onModuleStateChanged(String state, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onModuleModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub
		
	}

	public ModuleFactory getModuleFactory() {
		return moduleFactory;
	}
	public BlackboardHandler getBlackBoardHandler() {
		return blackboardHandler;
	}
}
