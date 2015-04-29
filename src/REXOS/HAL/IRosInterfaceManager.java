package HAL;

import org.json.JSONObject;

import HAL.steps.HardwareStep;

public interface IRosInterfaceManager {
	abstract void postHardwareStep(HardwareStep hardwareStep);
	abstract void postEquipletCommand(JSONObject equipletCommand);
	
	abstract void shutdown();
}
