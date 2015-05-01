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

public abstract class RosInterface {
	protected HardwareAbstractionLayer hal;
	private ArrayList<ModuleListener> moduleSubscribers;
	private ArrayList<EquipletListener> equipletSubscribers;
	private ArrayList<ProcessListener> processSubscribers;
	
	protected RosInterface(HardwareAbstractionLayer hal) {
		this.hal = hal;
		moduleSubscribers = new ArrayList<ModuleListener>();
		equipletSubscribers = new ArrayList<EquipletListener>();
		processSubscribers = new ArrayList<ProcessListener>();
	}
	
	public void addProcessListener(ProcessListener listener) {
		synchronized (processSubscribers) {
			processSubscribers.add(listener);
		}
	}
	public void removeProcessListener(ProcessListener listener) {
		synchronized (processSubscribers) {
			processSubscribers.remove(listener);
		}
	}
	protected void onHardwareStepStatusChanged(HardwareStep hardwareStep) {
		synchronized (processSubscribers) {
			for (ProcessListener processListener : processSubscribers) {
				processListener.onProcessStatusChanged(hardwareStep);
			}
		}
	}

	public void addEquipletListener(EquipletListener listener) {
		synchronized (equipletSubscribers) {
			equipletSubscribers.add(listener);
		}
	}
	public void removeEquipletListener(EquipletListener listener) {
		synchronized (equipletSubscribers) {
			equipletSubscribers.remove(listener);
		}
	}
	protected void onEquipletStateChanged(State state) {
		synchronized (equipletSubscribers) {
			for (EquipletListener equipletListener : equipletSubscribers) {
				equipletListener.onEquipletStateChanged(state);
			}
		}
	}
	protected void onEquipletModeChanged(Mode mode) {
		synchronized (equipletSubscribers) {
			for (EquipletListener equipletListener : equipletSubscribers) {
				equipletListener.onEquipletModeChanged(mode);
			}
		}
	}
	protected void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		synchronized (equipletSubscribers) {
			for (EquipletListener equipletListener : equipletSubscribers) {
				equipletListener.onEquipletCommandStatusChanged(status);
			}
		}
	}

	public void addModuleListener(ModuleListener listener) {
		synchronized (moduleSubscribers) {
			moduleSubscribers.add(listener);
		}
	}
	public void removeModuleListener(ModuleListener listener) {
		synchronized (moduleSubscribers) {
			moduleSubscribers.remove(listener);
		}
	}
	protected void onModuleStateChanged(Module module, State state) {
		synchronized (moduleSubscribers) {
			for (ModuleListener moduleListener : moduleSubscribers) {
				moduleListener.onModuleStateChanged(module, state);
			}
		}
	}
	protected void onModuleModeChanged(Module module, Mode mode) {
		synchronized (moduleSubscribers) {
			for (ModuleListener moduleListener : moduleSubscribers) {
				moduleListener.onModuleModeChanged(module, mode);
			}
		}
	}
	
	abstract public void postHardwareStep(HardwareStep hardwareStep);
	abstract public void postEquipletCommand(JSONObject equipletCommand);
	
	abstract public void shutdown();
}