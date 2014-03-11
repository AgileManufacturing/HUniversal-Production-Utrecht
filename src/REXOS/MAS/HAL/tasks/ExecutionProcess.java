package HAL.tasks;

import java.util.ArrayList;

import libraries.knowledgedb_client.KnowledgeException;
import HAL.HardwareStep;
import HAL.Module;
import HAL.ModuleFactory;
import HAL.exceptions.HardwareAbstractionLayerProcessException;
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
		} catch (InterruptedException | KnowledgeException ex) {
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
	@Override
	public void onModuleStateChanged(String state, Module module) {
		hardwareAbstractionLayerListener.onModuleStateChanged(state, module);
	}
	@Override
	public void onModuleModeChanged(String mode, Module module) {
		hardwareAbstractionLayerListener.onModuleModeChanged(mode, module);
	}
}
