package HAL;

import java.util.ArrayList;

import HAL.listeners.HardwareAbstractionLayerListener;

public class HALTesterClass implements HardwareAbstractionLayerListener {
	static HALTesterClass htc = new HALTesterClass();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		HardwareAbstractionLayer hal = new HardwareAbstractionLayer(htc);
		Service service = new Service("PickAndPlace");
		ProductStep productStep = new ProductStep(0, null, service);
		hal.translateProductStep(productStep);
	}

	@Override
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module, HardwareStep hardwareStep) {
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

	@Override
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareStep) {
		// TODO Auto-generated method stub
		hardwareSteps.addAll(hardwareStep);// = hardwareStep;
		
	}

	@Override
	public void onIncapableCapabilities(ProductStep productStep) {
		// TODO Auto-generated method stub
		
	}

}
