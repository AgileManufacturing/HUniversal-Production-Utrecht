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
import HAL.listeners.TestProcessListener;
import HAL.listeners.ViolationListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public class TestProcess implements Runnable, ExecutionProcessListener, EquipletCommandListener, ViolationListener {
	protected ShadowHardwareAbstractionLayer hal;
	protected ArrayList<HardwareStep> hardwareSteps;
	protected ProductStep productStep;
	protected TestProcessListener listener;
	protected boolean testFailed;
	
	public TestProcess(ShadowHardwareAbstractionLayer hal, ArrayList<HardwareStep> hardwareSteps, ProductStep productStep, TestProcessListener listener) {
		this.hal = hal;
		this.hardwareSteps = hardwareSteps;
		this.productStep = productStep;
		this.listener = listener;
		this.testFailed = false;
	}

	@Override
	public synchronized void run() {
		try{
			addParts();
			hal.getRosInterface().addViolationListener(this);
			
			hal.executeHardwareSteps(hardwareSteps, this);
			Logger.log(LogSection.HAL_TEST, LogLevel.DEBUG, "Waiting for part model to spawn");
			wait();
			removeParts();
			
			if(testFailed == false) listener.onTestFinished(productStep, hardwareSteps);
			else listener.onTestFailed(productStep);
		} catch (InterruptedException ex) {
			Logger.log(LogSection.HAL_TEST, LogLevel.WARNING, "Test process interrupted", ex);
			listener.onTestFailed(productStep);
		} catch (JSONException ex) {
			Logger.log(LogSection.HAL_TEST, LogLevel.ERROR, "Invalid product step", ex);
			listener.onTestFailed(productStep);
		} finally {
			hal.getRosInterface().removeViolationListener(this);
		}
	}
	protected JSONObject[] retrieveParts() throws JSONException {
		JSONArray subjects = productStep.getCriteria().getJSONArray("subjects");
		// also add part for target product
		JSONObject[] output = new JSONObject[subjects.length() + 1];
		for(int i = 0; i < subjects.length(); i++) {
			JSONObject subject = (JSONObject) subjects.get(i);
			output[i] = subject;
		}
		output[output.length - 1] = (JSONObject) productStep.getCriteria().getJSONObject("target");
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
					parameters.put("positionX", 35.0);
					parameters.put("positionY", 0.0);
					parameters.put("positionZ", 0.0);
				} else if(i == 1) {
					parameters.put("positionX", -35.0);
					parameters.put("positionY", 0.0);
					parameters.put("positionZ", 0.0);
				} else if(i == 2) {
					parameters.put("positionX", 0.0);
					parameters.put("positionY", 55.0);
					parameters.put("positionZ", 0.0);
				} else if(i == 3) {
					parameters.put("positionX", 0.0);
					parameters.put("positionY", -55.0);
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
		Thread.sleep(10000);
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
		testFailed = true;
		notify();
	}

	@Override
	public synchronized void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		if(status == EquipletCommandStatus.DONE || status == EquipletCommandStatus.FAILED) {
			notify();
		}
	}

	@Override
	public void onViolationOccured(ViolationType violationType, String message) {
		Logger.log(LogSection.HAL_TEST, LogLevel.DEBUG, "Violation occured of type " + violationType, message);
		testFailed = true;
	}
}
