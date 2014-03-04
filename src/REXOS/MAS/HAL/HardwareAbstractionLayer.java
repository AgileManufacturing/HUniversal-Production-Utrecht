package HAL;

import java.util.ArrayList;

import com.mongodb.util.JSON;

import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;
import HAL.tasks.ExecutionProcess;
import HAL.tasks.TranslationProcess;

public class HardwareAbstractionLayer implements ModuleListener{
	private CapabilityFactory capabilityFactory;
	private ModuleFactory moduleFactory;
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	
	public HardwareAbstractionLayer(HardwareAbstractionLayerListener hardwareAbstractionLayerListener){
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
	
	public boolean insertModule(ModuleIdentifier moduleIdentifier,JSON module, JSON dynamicSettings, JSON staticSettings){
		return moduleFactory.insertModule(moduleIdentifier, module, dynamicSettings, staticSettings);
	}
	public boolean updateModule(ModuleIdentifier moduleIdentifier,JSON module, JSON dynamicSettings){
		return moduleFactory.updateModule(moduleIdentifier, module, dynamicSettings);
	}
	public JSON deleteModule(ModuleIdentifier moduleIdentifier){
		return moduleFactory.deleteModule(moduleIdentifier);
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
