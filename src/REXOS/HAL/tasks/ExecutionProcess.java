package HAL.tasks;

import java.util.ArrayDeque;
import java.util.ArrayList;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.Module;
import HAL.ModuleActor;
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
	//private ArrayList<HardwareStep> hardwareSteps;
	private ModuleFactory moduleFactory;
	private int count =0;
	
	
	private HardwareStep currentStep = null;
	private ArrayDeque<HardwareStep> toExecuteSteps = new ArrayDeque<HardwareStep>();
	
	/**
	 * Constructs the ExecutionProcess but does NOT start it.
	 * @param hardwareAbstractionLayerListener
	 * @param hardwareSteps
	 * @param moduleFactory
	 */
	public ExecutionProcess(HardwareAbstractionLayerListener hardwareAbstractionLayerListener, ArrayList<HardwareStep> hardwareSteps, ModuleFactory moduleFactory){
		this.hardwareAbstractionLayerListener = hardwareAbstractionLayerListener;
		
		this.toExecuteSteps.clear();
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
			while(!toExecuteSteps.isEmpty()){
				currentStep = toExecuteSteps.poll();
				ModuleActor module = (ModuleActor) moduleFactory.getModuleByIdentifier(currentStep.getModuleIdentifier());
				module.executeHardwareStep(this, currentStep);
				Logger.log(LogSection.HAL_EXECUTION, LogLevel.DEBUG, "Wait for hardware step to finish: " + currentStep);
				
				this.wait();
				
				Logger.log(LogSection.HAL_EXECUTION, LogLevel.INFORMATION, "Hardware step finished");
			}
			currentStep = null;
		} catch (InterruptedException e) {
			e.printStackTrace();
		} finally {
			hardwareAbstractionLayerListener.onExecutionFinished();
			System.out.println("onExecutingFinished called!");
		}
	}
	
	/**
	 * This method handles the onProcessStateChanged events generated by the modules. It will notify the run method when necessary.
	 * Do not call this method! 
	 * @throws  
	 */
	@Override
	public synchronized void onProcessStatusChanged(HardwareStepStatus status, String hardwareStepSerialId, Module module) {
		//The onProcessStateChanged listener keeps listening even when there is no hardware step being executed,
		//Check if we should ignore this state change.
		if(currentStep == null){
			System.out.println("State changed after all steps have been executed or before any step is executed, ignore this state change!");
			return;
		}
		
		//The equals method is overriden in the ModuleIdentifier class, therefore the equals method works!
		if(!currentStep.getModuleIdentifier().equals(module.getModuleIdentifier())){
			System.out.println("Recieved a progressStateChange from another module, ignore this state change!");
			return;
		}
		
		
		
		//We have a hardware step to process and the module from the state change is the same as the one we are executing, proceed!
		System.out.println("Count: " + count  + "  " + module.getModuleIdentifier());
		if(status == HardwareStepStatus.DONE){
			count++;
			
			//if (hardwareSteps.size() > 0){
				HardwareStep hardwareStep = currentStep;
				hardwareAbstractionLayerListener.onProcessStatusChanged(status, module, hardwareStep);
				System.out.println("onProcessStateChanged: " + hardwareStep);
				
				//hardwareSteps.remove(0);
				//TODO MONGO FIX.. REMOVE ALL MONGO REFERENCES AND CLASSES! IT'S ASS FUCKING SLOWWWWW..!
				
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				    this.notify();
			/*
			}
			else {
				System.out.println("Count for the Throw: "+count);
				throw new RuntimeException("No more hardwareSteps while there should be at least one more");
			}
			*/
		}
		else if(status == HardwareStepStatus.FAILED) {
			Logger.log(LogSection.HAL_EXECUTION, LogLevel.ERROR, "Module is unable to execute hardwareStep");
			hardwareAbstractionLayerListener.onExecutionFailed();
		}
		//If status is WAITING or IN_PROGRESS
		else {
			//if (hardwareSteps.size() > 0){
				HardwareStep hardwareStep = currentStep;
				hardwareAbstractionLayerListener.onProcessStatusChanged(status, module, hardwareStep);	
			/*
		}
			else {
				System.out.println("HarwareSteps Done");
				//throw new RuntimeException("No more hardwareSteps while there should be at least one more");
			}
			*/
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
