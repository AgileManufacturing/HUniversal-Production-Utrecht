package HAL.modules;

import java.util.ArrayList;

import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.Module;
import HAL.listeners.ModuleListener;

public class PickAndPlace extends Module {

	

	public PickAndPlace(ModuleListener moduleListener) {
		super(moduleListener);
		// TODO Auto-generated constructor stub
	}

	@Override
	public void executeHardwareStep(HardwareStep hardwareStep) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(
			CompositeStep compositeStep) {
		// TODO Auto-generated method stub
		return null;
	}

}
