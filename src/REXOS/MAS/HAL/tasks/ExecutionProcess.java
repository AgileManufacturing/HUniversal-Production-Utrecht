package HAL.tasks;

import java.util.ArrayList;

import HAL.HardwareStep;
import HAL.Module;
import HAL.exceptions.HardwareAbstractionLayerProcessException;
import HAL.factories.ModuleFactory;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ProcessListener;

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
				System.out.println("Executing Hardwarestep: "+hardwareSteps.get(0));
				moduleFactory.executeHardwareStep(this, hardwareSteps.get(0));
				this.wait();
				System.out.println("resumed");
			}
			hardwareAbstractionLayerListener.onExecutionFinished();
		} catch (Exception ex) {
			// TODO Auto-generated catch block
			ex.printStackTrace();
		}
	}

	@Override
	public synchronized void onProcessStatusChanged(String state, long hardwareStepSerialId, Module module) throws HardwareAbstractionLayerProcessException{
		System.out.println("Hardware step completed!");
		if (hardwareSteps.size() > 0){
			HardwareStep hardwareStep = hardwareSteps.get(0);
			hardwareAbstractionLayerListener.onProcessStateChanged(state, hardwareStepSerialId, module, hardwareStep);
			hardwareSteps.remove(0);
			System.out.println("Notify");
			this.notify();
		}
		else {
			throw new HardwareAbstractionLayerProcessException("No more hardwareSteps while there should be at least one more");
		}
	}
}
