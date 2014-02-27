package HAL.modules;

import java.util.ArrayList;


import HAL.CompositeStep;
import HAL.HardwareStep;
import HAL.Module;

public class Gripper extends Module {

	@Override
	protected void setRequiredAttributes() {
		typeNumber = "HU_gripper_type_a";
		manufacturer = "HU";
	}

	@Override
	public void executeHardwareStep(HardwareStep hardwareStep) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public ArrayList<HardwareStep> translateCompositeStep(CompositeStep compositeStep) {
		// TODO Auto-generated method stub
		return null;
	}
}
