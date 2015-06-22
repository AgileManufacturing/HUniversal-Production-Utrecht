package HAL;

import generic.Mast.Mode;
import generic.Mast.State;

import java.util.ArrayList;

import org.json.JSONObject;

import HAL.dataTypes.ModuleIdentifier;
import HAL.factories.ModuleFactory;
import HAL.listeners.EquipletCommandListener.EquipletCommandStatus;
import HAL.listeners.EquipletListener;
import HAL.listeners.ModuleListener;
import HAL.listeners.ProcessListener;
import HAL.listeners.ViolationListener;
import HAL.listeners.ViolationListener.ViolationType;
import HAL.steps.HardwareStep;

public abstract class RosInterface {
	private ArrayList<EquipletListener> equipletSubscribers;
	private ArrayList<ProcessListener> processSubscribers;
	private ArrayList<ViolationListener> violationSubscribers;
	protected ModuleFactory moduleFactory;
	
	protected RosInterface(ModuleFactory moduleFactory) {
		this.moduleFactory = moduleFactory;
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

	protected void onModuleStateChanged(ModuleIdentifier module, State state) {
		moduleFactory.getItemForIdentifier(module).onModuleStateChanged(module, state);
	}
	protected void onModuleModeChanged(ModuleIdentifier module, Mode mode) {
		moduleFactory.getItemForIdentifier(module).onModuleModeChanged(module, mode);
	}
	
	public void addViolationListener(ViolationListener listener) {
		synchronized (violationSubscribers) {
			violationSubscribers.add(listener);
		}
	}
	public void removeViolationListener(ViolationListener listener) {
		synchronized (violationSubscribers) {
			violationSubscribers.remove(listener);
		}
	}
	public void onViolationOccured(ViolationType violationType, String message) {
		synchronized (violationSubscribers) {
			for (ViolationListener violationListener : violationSubscribers) {
				violationListener.onViolationOccured(violationType, message);
			}
		}
	}
	
	abstract public void postHardwareStep(HardwareStep hardwareStep);
	abstract public void postEquipletCommand(JSONObject equipletCommand);
	
	abstract public void shutdown();
}