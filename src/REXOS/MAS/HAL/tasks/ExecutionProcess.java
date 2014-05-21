package HAL.tasks;

import java.util.ArrayList;

import HAL.Module;
import HAL.exceptions.HardwareAbstractionLayerProcessException;
import HAL.factories.ModuleFactory;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ProcessListener;
import HAL.steps.HardwareStep;

public class ExecutionProcess implements Runnable, ProcessListener{
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private ArrayList<HardwareStep> hardwareSteps;
	private ModuleFactory moduleFactory;
	
	public ExecutionProcess(HardwareAbstractionLayerListener hardwareAbstractionLayerListener, ArrayList<HardwareStep> hardwareSteps, ModuleFactory moduleFactory){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		this.hardwareSteps = new ArrayList<HardwareStep>();
		this.hardwareSteps.addAll(hardwareSteps);
		this.moduleFactory = moduleFactory;
	}

	@Override
	public synchronized void run() {
		try {
			while (hardwareSteps.size() > 0){
				moduleFactory.executeHardwareStep(this, hardwareSteps.get(0));
				this.wait();
			}
			hardwareAbstractionLayerListener.onExecutionFinished();
		} catch (Exception ex) {
			// TODO Auto-generated catch block
			ex.printStackTrace();
		}
	}

	@Override
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module) throws HardwareAbstractionLayerProcessException{
		if (hardwareSteps.size() > 0){
			HardwareStep hardwareStep = hardwareSteps.get(0);
			hardwareAbstractionLayerListener.onProcessStateChanged(state, hardwareStepSerialId, module, hardwareStep);
			hardwareSteps.remove(0);
			this.notify();
		}
		else {
			throw new HardwareAbstractionLayerProcessException("No more hardwareSteps while there should be at least one more");
		}
	}

	public void start() {
		Thread t = new Thread(this);
		t.start();
	}
}
