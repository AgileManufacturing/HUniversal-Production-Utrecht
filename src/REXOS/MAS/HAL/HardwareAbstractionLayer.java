package HAL;

import java.util.ArrayList;

import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;

public class HardwareAbstractionLayer implements ModuleListener {
	public HardwareAbstractionLayer(HardwareAbstractionLayerListener hardwareAbstractionLayerListener){
		
	}
	public void executeProductStep(ProductStep productStep){
		
	}
	public ArrayList<HardwareStep> translateProductStep(ProductStep productStep){
		return null;		
	}
	public ArrayList<Capability> getAllCapabilities(){
		return null;
	}
	public void onProcessStateChanged(String state, long hardwareStepSerialId){
		
	}
	public void onModuleStateChanged(String state, Module module){
		
	}
	public void onModuleModeChanged(String state, Module module){
		
	}
}
