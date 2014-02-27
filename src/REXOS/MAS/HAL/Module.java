package HAL;

import java.util.ArrayList;

import HAL.listeners.ModuleListener;

public class Module {
	public Module(ModuleListener moduleListener){
		
	}
	private Module getParentModule(){
		return null;
	}
	public void executeHardwareStep(HardwareStep hardwareStep){
		
	}
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep){
		return null;
	}
	public void onHardwareStepChanged(String state, long hardwareStepSerialId){
		
	}
	public void onModuleStateChanged(String state){
		
	}
	public void onModuleModeChanged(String mode){
		
	}
}
