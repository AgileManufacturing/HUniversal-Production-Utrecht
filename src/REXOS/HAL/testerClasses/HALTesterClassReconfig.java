package HAL.testerClasses;

import generic.Mast.Mode;
import generic.Mast.State;

import java.util.ArrayList;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.dataTypes.ModuleIdentifier;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public class HALTesterClassReconfig implements HardwareAbstractionLayerListener {
	static HALTesterClassReconfig htc = new HALTesterClassReconfig();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	
	public static void main(String[] args) throws Exception {
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "Starting");
		
		hal = new HardwareAbstractionLayer("HALTesterClassReconfig", htc);
		
		// TODO add reconfigigureEquiplet in HAL again
		//hal.reconfigureEquiplet();
	}
	

	// TODO CHANGE THIS: not available in HAL
	
	//@Override
	public String getEquipletName() {
		// TODO hardcoded!!!!!!
		
		return "EQ2";
	}

	@Override
	public void onExecutionFinished() {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Execution finished");
	}



	@Override
	public void onExecutionFailed() {
		// TODO Auto-generated method stub
		Logger.log(LogSection.NONE, LogLevel.ERROR, "Execution FAILED!");
	}

	/**
	 * [onReloadEquiplet -Test function W.I.P (Lars Veenendaal)]
	 * @param state [description]
	 */
	// TODO CHANGE THIS: not available in HAL
	
	//@Override
	public void onReloadEquiplet(String state){
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Reloading has: " + state);

	}

	@Override
	public void onEquipletStateChanged(State state) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of equiplet " + getEquipletName() + " has changed to " + state);
	}

	@Override
	public void onEquipletModeChanged(Mode mode) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The mode of module " + mode.name() + " has changed to " + mode);		
	}

	// TODO CHANGE THIS: empty
	@Override
	public void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onModuleStateChanged(ModuleIdentifier module, State state) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of module " + module + " has changed to " + state);
	}

	// TODO CHANGE THIS: empty
	@Override
	public void onModuleModeChanged(ModuleIdentifier module, Mode mode) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onTranslationFinished(ProductStep productStep,
			ArrayList<HardwareStep> hardwareSteps) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hardwareSteps.addAll(hardwareSteps);// = hardwareStep;
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onTranslationFailed(ProductStep productStep) {
		Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, "Translation failed of the following product step:", new Object[] { productStep.getService(), productStep.getCriteria() });
		
	}

	// TODO CHANGE THIS: status wasn't available
	@Override
	public void onProcessStatusChanged(Module module, HardwareStep hardwareStep) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The status of " + hardwareStep + " (being processed by module " + module + ") has changed to " + "CHANGE THIS TO STATUS");
	}
}
