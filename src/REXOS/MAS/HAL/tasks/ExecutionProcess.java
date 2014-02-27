package HAL.tasks;

import java.util.ArrayList;

import HAL.HardwareStep;
import HAL.Module;
import HAL.ModuleFactory;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ModuleListener;

public class ExecutionProcess implements Runnable, ModuleListener{
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private ArrayList<HardwareStep> hardwareSteps;
	
	public ExecutionProcess(HardwareAbstractionLayerListener hardwareAbstractionLayerListener, ArrayList<HardwareStep> hardwareSteps){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		this.hardwareSteps = new ArrayList<HardwareStep>();
		this.hardwareSteps.addAll(hardwareSteps);
	}

	@Override
	public void run() {
		try {
			ModuleFactory moduleFactory = new ModuleFactory(this);
			while (hardwareSteps.size() > 0){
				moduleFactory.executeHardwareStep(hardwareSteps.get(0));
				this.wait();
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module) {
		if (hardwareSteps.size() > 0){
			HardwareStep hardwareStep = hardwareSteps.get(0);
			hardwareAbstractionLayerListener.onProcessStateChanged(state, hardwareStepSerialId, module, hardwareStep);
			hardwareSteps.remove(0);
			this.notify();
		}
		else {
			//TODO HALListener call processStep error?
		}
	}
	@Override
	public void onModuleStateChanged(String state, Module module) {
		hardwareAbstractionLayerListener.onModuleStateChanged(state, module);
	}
	@Override
	public void onModuleModeChanged(String mode, Module module) {
		hardwareAbstractionLayerListener.onModuleModeChanged(mode, module);
	}
}
