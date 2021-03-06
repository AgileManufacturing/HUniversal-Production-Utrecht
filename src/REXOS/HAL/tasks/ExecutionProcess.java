package HAL.tasks;

import java.util.ArrayDeque;
import java.util.ArrayList;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.ModuleActor;
import HAL.factories.ModuleFactory;
import HAL.listeners.ExecutionProcessListener;
import HAL.listeners.ProcessListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
/**
 * The thread that manages the execution of hardware steps
 * @author Bas Voskuijlen
 *
 */
public class ExecutionProcess implements Runnable, ProcessListener{
	private ExecutionProcessListener listener;
	private ModuleFactory moduleFactory;
	
	private HardwareStep currentStep = null;
	private ArrayDeque<HardwareStep> toExecuteSteps = new ArrayDeque<HardwareStep>();
	private boolean continueExecution = true;
	
	/**
	 * Constructs the ExecutionProcess but does NOT start it.
	 * @param hardwareAbstractionLayerListener
	 * @param hardwareSteps
	 * @param moduleFactory
	 */
	public ExecutionProcess(ExecutionProcessListener listener, ArrayList<HardwareStep> hardwareSteps, ModuleFactory moduleFactory){
		this.listener = listener;
		
		this.toExecuteSteps.addAll(hardwareSteps);
		this.moduleFactory = moduleFactory;
	}

	/**
	 * This method will execute the HardwareSteps in chronological order.
	 * This method returns after all HardwareSteps for this ExecutionProcess have been executed and therefore should be called asynchronous. This should be done with the start method.
	 */
	@Override
	public synchronized void run() {
		Logger.log(LogSection.HAL_EXECUTION, LogLevel.INFORMATION, "Execution started with the following hardware steps:", toExecuteSteps);
		
		try {
			while(true){
				if(currentStep != null) {
					// we just finished executing a step, deregister from module
					ModuleActor previousModule = (ModuleActor) moduleFactory.getItemForIdentifier(currentStep.getModuleIdentifier());
					previousModule.removeProcessListener(this);
					if(continueExecution == false || toExecuteSteps.isEmpty() == true) {
						currentStep = null;
						break;
					}
				}
				currentStep = toExecuteSteps.poll();
				ModuleActor module;
				module = (ModuleActor) moduleFactory.getItemForIdentifier(currentStep.getModuleIdentifier());
				module.addProcessListener(this);
				module.executeHardwareStep(currentStep);
				Logger.log(LogSection.HAL_EXECUTION, LogLevel.DEBUG, "Wait for hardware step to finish: " + currentStep);
				
				this.wait();
				
				Logger.log(LogSection.HAL_EXECUTION, LogLevel.INFORMATION, "Hardware step finished");
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		} finally {
			if(continueExecution == true) {
				listener.onExecutionFinished();
			} else {
				listener.onExecutionFailed();
			}
		}
	}
	
	/**
	 * This method handles the onProcessStateChanged events generated by the modules. It will notify the run method when necessary.
	 * Do not call this method! 
	 * @throws  
	 */
	@Override
	public synchronized void onProcessStatusChanged(HardwareStep hardwareStep) {
		if(hardwareStep != currentStep) {
			Logger.log(LogSection.HAL_EXECUTION, LogLevel.ERROR, 
					"Recieved a progressStatusChanged from hardware step that is not being executed");
		}
		ModuleActor module = (ModuleActor) moduleFactory.getItemForIdentifier(hardwareStep.getModuleIdentifier());
		listener.onProcessStatusChanged(module, hardwareStep);
		
		if(hardwareStep.getStatus() == HardwareStepStatus.DONE) {
		    this.notify();
		} else if(hardwareStep.getStatus() == HardwareStepStatus.FAILED) {
			Logger.log(LogSection.HAL_EXECUTION, LogLevel.ERROR, "Module is unable to execute hardwareStep");
			continueExecution = false;
		    this.notify();
		} else {
			// ignore other process status changes
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
