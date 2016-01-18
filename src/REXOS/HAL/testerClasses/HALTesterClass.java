package HAL.testerClasses;

import generic.Mast;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.StaticSettings;
import HAL.exceptions.BlackboardUpdateException;
import HAL.exceptions.InvalidMastModeException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public class HALTesterClass implements HardwareAbstractionLayerListener {
	HardwareAbstractionLayer hal;
	
	static String equipletName = "EQ0";
	static final String baseDir = "generatedOutput/";
	static boolean insertModules = true;
	static boolean translateSteps = true;
	static int serialNumber = 1;
	
	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		if(args.length >= 1) {
			if(args[0].equals("--noInsert")) {
				insertModules = false;
			} else if(args[0].equals("--noTranslate")) {
				translateSteps = false;
			}
		}
		
		if(args.length >= 3) {
			equipletName = args[1];
			serialNumber = Integer.parseInt(args[2]);
		} if(args.length >= 2) {
			equipletName = args[1];
		} else if(args.length >= 1) {
			equipletName = args[0];
		}
		System.out.println("Inserting equiplet " + equipletName);
		
		@SuppressWarnings("unused")
		HALTesterClass htc = new HALTesterClass();
		htc = null;
	}
	public HALTesterClass() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException, ParseException {
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "Starting");
	}
	
	protected StaticSettings getStaticSettings(ModuleIdentifier moduleIdentifier) throws JSONException, FileNotFoundException, ParseException {
		FileInputStream fileInputStream = new FileInputStream(new File(getStaticSettingsPath(moduleIdentifier)));
		JSONTokener jsonTokener = new JSONTokener(fileInputStream);
		JSONObject jsonObject = new JSONObject(jsonTokener);
		return StaticSettings.deSerialize(jsonObject);
	}
	protected String getStaticSettingsPath(ModuleIdentifier moduleIdentifier) {
		return baseDir + "staticSettings/"
				+ moduleIdentifier.manufacturer + "/"
				+ moduleIdentifier.typeNumber + "/"
				+ moduleIdentifier.serialNumber + "/"
				+ "staticSettings.json";
	}
	@Override
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareSteps) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onTranslationFailed(ProductStep productStep) {
		Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, equipletName + " Translation failed of the following product step:", productStep);
	}

	@Override
	public void onProcessStatusChanged(Module module, HardwareStep hardwareStep) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The status of " + hardwareStep + " (being processed by module " + module + ") has changed to " + hardwareStep.getStatus());
	}

	@Override
	public void onModuleStateChanged(ModuleIdentifier module, Mast.State state) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of module " + module + " has changed to " + state);
	}

	@Override
	public void onModuleModeChanged(ModuleIdentifier module, Mast.Mode mode) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The mode of module " + module + " has changed to " + mode);
	}

	@Override
	public void onExecutionFinished() {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Execution finished");
	}

	@Override
	public void onEquipletStateChanged(Mast.State state) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of equiplet " + equipletName + " has changed to " + state);
		if(state == Mast.State.SAFE) {
			hal.changeState(Mast.State.STANDBY);
		} else if(state == Mast.State.STANDBY) {
			hal.changeMode(Mast.Mode.NORMAL);
		}
	}

	@Override
	public void onEquipletModeChanged(Mast.Mode mode) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The mode of equiplet " + equipletName + " has changed to " + mode);
	}

	@Override
	public void onExecutionFailed() {
		Logger.log(LogSection.NONE, LogLevel.ERROR, equipletName + " Execution failed");
	}

	@Override
	public void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Reloading has: " + status);
	}
	public void onEquipletCommandReply(JSONObject reply){
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Reply on equipletcommand");
	}
}
