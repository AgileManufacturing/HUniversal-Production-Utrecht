package HAL;

import java.util.ArrayList;

import HAL.listeners.ModuleListener;

public abstract class Module { //implements mongolistener
	private ModuleListener moduleListener;
	
	public Module(ModuleListener moduleListener){
		this.moduleListener = moduleListener;
	}
	@SuppressWarnings("unused")
	private Module getParentModule(){
		//TODO
		return null;
	}
	
	abstract public void executeHardwareStep(HardwareStep hardwareStep);
	abstract public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep);
	
	public void onHardwareStepChanged(String state, long hardwareStepSerialId){
		moduleListener.onProcessStateChanged(state, hardwareStepSerialId, this);
	}
	public void onModuleStateChanged(String state){
		moduleListener.onModuleStateChanged(state, this);
	}
	public void onModuleModeChanged(String mode){
		moduleListener.onModuleModeChanged(mode, this);
	}
}
