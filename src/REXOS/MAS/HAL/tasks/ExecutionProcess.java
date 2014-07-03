package HAL.tasks;

import java.util.ArrayList;

import libraries.log.LogLevel;
import libraries.log.LogSection;
import libraries.log.Logger;
import HAL.Module;
import HAL.exceptions.FactoryException;
import HAL.factories.ModuleFactory;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.listeners.ProcessListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
/**
 * The thread that manages the execution of hardware steps
 * @author Bas Voskuijlen
 *
 */
public class ExecutionProcess implements Runnable, ProcessListener{
	private HardwareAbstractionLayerListener hardwareAbstractionLayerListener;
	private ArrayList<HardwareStep> hardwareSteps;
	private ModuleFactory moduleFactory;
	/**
	 * Constructs the ExecutionProcess but does NOT start it.
	 * @param hardwareAbstractionLayerListener
	 * @param hardwareSteps
	 * @param moduleFactory
	 */
	public ExecutionProcess(HardwareAbstractionLayerListener hardwareAbstractionLayerListener, ArrayList<HardwareStep> hardwareSteps, ModuleFactory moduleFactory){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		this.hardwareSteps = new ArrayList<HardwareStep>();
		this.hardwareSteps.addAll(hardwareSteps);
		this.moduleFactory = moduleFactory;
	}

	/**
	 * This method will execute the HardwareSteps in chronological order.
	 * This method returns after all HardwareSteps for this ExecutionProcess have been executed and therefore should be called asynchronous. This should be done with the start method.
	 */
	@Override
	public synchronized void run() {
		Logger.log(LogSection.HAL_EXECUTION, LogLevel.INFORMATION, "Execution started with the following hardware steps:", hardwareSteps);
		
		System.out.println("hardwareSteps size: "+ hardwareSteps.size());
		for(int i =0 ; i<hardwareSteps.size();i++){
			try {
				System.out.println("Executing: "+hardwareSteps.get(i));
				moduleFactory.executeHardwareStep(this, hardwareSteps.get(i));
				//this.wait();
				System.out.println("After the wait");
			} catch (FactoryException e) {
				Logger.log(LogSection.HAL_EXECUTION, LogLevel.ERROR, "Module is unable to execute hardwareStep: " + e);
			}
		}
		/*while (hardwareSteps.size() > 0){
			try {
				System.out.println("Executing: "+hardwareSteps.get(0));
				moduleFactory.executeHardwareStep(this, hardwareSteps.get(0));
				//this.wait();
				System.out.println("After the wait");
			} catch (FactoryException e) {
				Logger.log(LogSection.HAL_EXECUTION, LogLevel.ERROR, "Module is unable to execute hardwareStep: " + e);
			}
		}*/
		hardwareAbstractionLayerListener.onExecutionFinished();
	}
	
	/**
	 * This method handles the onProcessStateChanged events generated by the modules. It will notify the run method when necessary.
	 * Do not call this method! 
	 * @throws  
	 */
	@Override
	public synchronized void onProcessStateChanged(String status, long hardwareStepSerialId, Module module) {
		Logger.log(LogSection.HAL_EXECUTION, LogLevel.DEBUG, "The status of hardwareStep identified by " + 
				hardwareStepSerialId + " (being processed by module " + module + ") has changed to " + status);
		if(status.equals(HardwareStepStatus.DONE)){
			if (hardwareSteps.size() > 0){
				HardwareStep hardwareStep = hardwareSteps.get(0);
				hardwareAbstractionLayerListener.onProcessStatusChanged(status, module, hardwareStep);
				hardwareSteps.remove(0);
				
				//TODO MONGO FIX.. REMOVE ALL MONGO REFERENCES AND CLASSES! IT'S ASS FUCKING SLOWWWWW..!
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				this.notify();
			}
			else {
				throw new RuntimeException("No more hardwareSteps while there should be at least one more");
			}
		}
		else if(status.equals(HardwareStepStatus.FAILED)) {
			Logger.log(LogSection.HAL_EXECUTION, LogLevel.ERROR, "Module is unable to execute hardwareStep");
			hardwareAbstractionLayerListener.onExecutionFailed();
		}
		else {
			if (hardwareSteps.size() > 0){
				HardwareStep hardwareStep = hardwareSteps.get(0);
				hardwareAbstractionLayerListener.onProcessStatusChanged(status, module, hardwareStep);	
			}
			else {
				throw new RuntimeException("No more hardwareSteps while there should be at least one more");
			}
		}
	}
	
	/**
	 * This method starts the ExecutionProcess asynchronously.
	 */
	public void start() {
		Thread t = new Thread(this);
		t.start();
	}
}
