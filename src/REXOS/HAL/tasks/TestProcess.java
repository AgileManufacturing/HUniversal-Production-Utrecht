package HAL.tasks;

import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.Module;
import HAL.ShadowHardwareAbstractionLayer;
import HAL.listeners.EquipletCommandListener;
import HAL.listeners.ExecutionProcessListener;
import HAL.steps.HardwareStep;

public class TestProcess implements Runnable, ExecutionProcessListener, EquipletCommandListener {
	protected ShadowHardwareAbstractionLayer hal;
	protected ArrayList<HardwareStep> hardwareSteps;
	protected JSONObject criteria;
	
	public TestProcess(ShadowHardwareAbstractionLayer hal, ArrayList<HardwareStep> hardwareSteps, JSONObject criteria) {
		this.hal = hal;
		this.hardwareSteps = hardwareSteps;
		this.criteria = criteria;
	}

	@Override
	public synchronized void run() {
		try{
			addParts();
			hal.executeHardwareSteps(hardwareSteps, this);
			Logger.log(LogSection.HAL_TEST, LogLevel.DEBUG, "Waiting for part model to spawn");
			wait();
			removeParts();
		} catch (InterruptedException ex) {
			Logger.log(LogSection.HAL_TEST, LogLevel.WARNING, "Test process interrupted", ex);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_TEST, LogLevel.ERROR, "Invalid product step", ex);
		}
	}
	protected JSONObject[] retrieveParts() throws JSONException {
		JSONArray subjects = criteria.getJSONArray("subjects");
		// also add part for target product
		JSONObject[] output = new JSONObject[subjects.length() + 1];
		for(int i = 0; i < subjects.length(); i++) {
			JSONObject subject = (JSONObject) subjects.get(i);
			output[i] = subject;
		}
		output[output.length - 1] = (JSONObject) criteria.getJSONObject("target");
		return output;
	}

	private synchronized void addParts() throws JSONException, InterruptedException {
		JSONObject[] parts = retrieveParts();
		
		for(int i = 0; i < parts.length; i++) {
			try{
				JSONObject subject = parts[i];
				
				JSONObject equipletCommand = new JSONObject();
				equipletCommand.put("command", "spawnPartModel");
				equipletCommand.put("status", "WAITING");
				JSONObject parameters = new JSONObject();
				parameters.put("partName", subject.getString("identifier"));
				parameters.put("originPlacementType", "RELATIVE_TO_MODULE_ORIGIN");
				// TODO hardcoding, retrieve location from transport unit or something?
				if(i == 0) {
					parameters.put("positionX", 50.0);
					parameters.put("positionY", 0.0);
					parameters.put("positionZ", 0.0);
				} else if(i == 1) {
					parameters.put("positionX", 0.0);
					parameters.put("positionY", 50.0);
					parameters.put("positionZ", 0.0);
				} else if(i == 2) {
					parameters.put("positionX", -50.0);
					parameters.put("positionY", 0.0);
					parameters.put("positionZ", 0.0);
				} else if(i == 3) {
					parameters.put("positionX", 0.0);
					parameters.put("positionY", -50.0);
					parameters.put("positionZ", 0.0);
				}
				// TODO hardcoding, determine moduleId by searching for a mutation support?
				parameters.put("relativeTo", "HU|workplane_type_A|1");
				parameters.put("rotationX", 0.0);
				parameters.put("rotationY", 0.0);
				parameters.put("rotationZ", 0.0);
				parameters.put("spawnChildParts", true);
				
				equipletCommand.put("parameters", parameters);
				
				hal.spawnPartModel(equipletCommand, this);
				Logger.log(LogSection.HAL_TEST, LogLevel.DEBUG, "Waiting for part model to spawn");
				wait();
			} catch(JSONException ex) {
				Logger.log(LogSection.HAL, LogLevel.EMERGENCY, "Error occured which is considered to be impossible", ex);
			}
		}
	}

	private synchronized void removeParts() {
		
		
	}

	/**
	 * This method starts the TestProcess asynchronously.
	 */
	public void start() {
		Thread t = new Thread(this);
		t.start();
	}

	@Override
	public void onProcessStatusChanged(Module module, HardwareStep hardwareStep) {
		// ignore
	}

	@Override
	public synchronized void onExecutionFinished() {
		notify();
	}

	@Override
	public synchronized void onExecutionFailed() {
		notify();
	}

	@Override
	public synchronized void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		if(status == EquipletCommandStatus.DONE || status == EquipletCommandStatus.FAILED) {
			notify();
		}
	}
}
