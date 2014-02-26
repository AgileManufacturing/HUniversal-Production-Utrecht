package HAL;

import java.util.ArrayList;

public class Module {
	public Module(HardwareAbstractionLayer hardwareAbstractionLayer){
		
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
