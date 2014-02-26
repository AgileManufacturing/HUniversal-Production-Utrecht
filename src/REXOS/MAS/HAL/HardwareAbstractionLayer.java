package HAL;

import java.util.ArrayList;
import agents.equiplet_agent.EquipletAgent;

public class HardwareAbstractionLayer {
	public HardwareAbstractionLayer(EquipletAgent equipletAgent){
		
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
