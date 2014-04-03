package HAL;

import java.util.ArrayList;

import com.google.gson.JsonObject;

import libraries.knowledgedb_client.KnowledgeException;
import HAL.factories.CapabilityFactory;
import HAL.factories.ModuleFactory;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;
import HAL.tasks.ExecutionProcess;
import HAL.tasks.TranslationProcess;

public class HardwareAbstractionLayer implements ModuleListener{
	private CapabilityFactory capabilityFactory;
	private ModuleFactory moduleFactory;
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	
	// TODO move somewhere else?? 
	private String equipletName;
	public String getEquipletName() {
		return equipletName;
	}

	public HardwareAbstractionLayer(HardwareAbstractionLayerListener hardwareAbstractionLayerListener) throws KnowledgeException{
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		capabilityFactory = new CapabilityFactory(this);
		moduleFactory = new ModuleFactory(this, this);
	}
	
	public void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps){
		ExecutionProcess executionProcess = new ExecutionProcess(this.hardwareAbstractionLayerListener, hardwareSteps);
		executionProcess.run();
	}
	public void translateProductStep(ProductStep productStep){
		TranslationProcess translationProcess = new TranslationProcess(this.hardwareAbstractionLayerListener, productStep);
		translationProcess.run();
	}
	public ArrayList<Capability> getAllCapabilities() throws Exception{
		return capabilityFactory.getAllSupportedCapabilities();
	}
	
	public boolean insertModule(JsonObject dynamicSettings, JsonObject staticSettings){
		return moduleFactory.insertModule(dynamicSettings, staticSettings);
	}
	public boolean updateModule(JsonObject dynamicSettings){
		return moduleFactory.updateModule(dynamicSettings);
	}
	public JsonObject deleteModule(ModuleIdentifier moduleIdentifier){
		return moduleFactory.deleteModule(moduleIdentifier);
	}
	public ArrayList<Module> getBottomModules() throws Exception{
		return moduleFactory.getBottomModules();
	}

	@Override
	public void onProcessStateChanged(String state, long hardwareStepSerialId,
			Module module) {
		// TODO Auto-generated method stub
	}

	@Override
	public void onModuleStateChanged(String state, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onModuleModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub
		
	}
}
