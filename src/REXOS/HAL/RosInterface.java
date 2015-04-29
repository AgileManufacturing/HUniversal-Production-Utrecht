package HAL;

import generic.Mast.Mode;
import generic.Mast.State;

import java.util.ArrayList;

import org.json.JSONObject;

import HAL.listeners.EquipletListener;
import HAL.listeners.EquipletListener.EquipletCommandStatus;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;
import HAL.steps.HardwareStep;

public class RosInterface {
	private HardwareAbstractionLayer hal;
	private IRosInterfaceManager manager;
	
	private ArrayList<ModuleListener> moduleSubscribers;
	private ArrayList<EquipletListener> equipletSubscribers;
	private ArrayList<ProcessListener> processSubscribers;
	
	public RosInterface(HardwareAbstractionLayer hal) {
		this.hal = hal;
		this.manager = new RosJavaNode(this);
		
		moduleSubscribers = new ArrayList<ModuleListener>();
		equipletSubscribers = new ArrayList<EquipletListener>();
		processSubscribers = new ArrayList<ProcessListener>();
	}
	
	public void addProcessListener(ProcessListener listener) {
		processSubscribers.add(listener);
	}
	public void removeProcessListener(ProcessListener listener) {
		processSubscribers.remove(listener);
	}
	public void onHardwareStepStatusChanged(HardwareStep hardwareStep) {
		for (ProcessListener processListener : processSubscribers) {
			processListener.onProcessStatusChanged(hardwareStep);
		}
	}

	public void addEquipletListener(EquipletListener listener) {
		equipletSubscribers.add(listener);
	}
	public void removeEquipletListener(EquipletListener listener) {
		equipletSubscribers.remove(listener);
	}
	public void onEquipletStateChanged(State state) {
		for (EquipletListener equipletListener : equipletSubscribers) {
			equipletListener.onEquipletStateChanged(state);
		}
	}
	public void onEquipletModeChanged(Mode mode) {
		for (EquipletListener equipletListener : equipletSubscribers) {
			equipletListener.onEquipletModeChanged(mode);
		}
	}
	public void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		for (EquipletListener equipletListener : equipletSubscribers) {
			equipletListener.onEquipletCommandStatusChanged(status);
		}
	}

	public void addModuleListener(ModuleListener listener) {
		moduleSubscribers.add(listener);
	}
	public void removeModuleListener(ModuleListener listener) {
		moduleSubscribers.remove(listener);
	}
	public void onModuleStateChanged(Module module, State state) {
		for (ModuleListener moduleListener : moduleSubscribers) {
			moduleListener.onModuleStateChanged(module, state);
		}
	}
	public void onModuleModeChanged(Module module, Mode mode) {
		for (ModuleListener moduleListener : moduleSubscribers) {
			moduleListener.onModuleModeChanged(module, mode);
		}
	}
	
	void postHardwareStep(HardwareStep hardwareStep) {
		manager.postHardwareStep(hardwareStep);
	}
	void postEquipletCommand(JSONObject equipletCommand) {
		manager.postEquipletCommand(equipletCommand);
	}
	
	void shutdown() {
		manager.shutdown();
	}

	public HardwareAbstractionLayer getHal() {
		return hal;
	}
}