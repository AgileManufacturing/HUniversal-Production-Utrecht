package HAL;

import java.util.ArrayList;

import com.google.gson.JsonObject;

import libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;
import HAL.tasks.ExecutionProcess;
import HAL.tasks.TranslationProcess;

public class HardwareAbstractionLayer implements ModuleListener{
	private CapabilityFactory capabilityFactory;
	private ModuleFactory moduleFactory;
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	
	public HardwareAbstractionLayer(HardwareAbstractionLayerListener hardwareAbstractionLayerListener) throws KnowledgeException{
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		capabilityFactory = new CapabilityFactory();
		moduleFactory = new ModuleFactory(this);
	}
	
	public void executeHardwareSteps(ArrayList<HardwareStep> hardwareSteps){
		ExecutionProcess executionProcess = new ExecutionProcess(this.hardwareAbstractionLayerListener, hardwareSteps);
		executionProcess.run();
	}
	public void translateProductStep(ProductStep productStep){
		TranslationProcess translationProcess = new TranslationProcess(this.hardwareAbstractionLayerListener, productStep);
		translationProcess.run();
	}
	public ArrayList<Capability> getAllCapabilities(){
		return capabilityFactory.getAllCapabilities();
	}
	
	public boolean insertModule(ModuleIdentifier moduleIdentifier, JsonObject rosSoftware, JsonObject module, JsonObject dynamicSettings, JsonObject staticSettings){
		return moduleFactory.insertModule(moduleIdentifier, rosSoftware, module, dynamicSettings, staticSettings);
	}
	public boolean updateModule(ModuleIdentifier moduleIdentifier, JsonObject rosSoftware, JsonObject module, JsonObject dynamicSettings){
		return moduleFactory.updateModule(moduleIdentifier, rosSoftware, module, dynamicSettings);
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
