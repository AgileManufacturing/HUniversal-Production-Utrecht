package HAL.testerClasses;

import java.io.IOException;
import java.text.ParseException;

import org.json.JSONException;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.dataTypes.DynamicSettings;
import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.StaticSettings;
import HAL.exceptions.BlackboardUpdateException;
import HAL.exceptions.InvalidMastModeException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.HardwareAbstractionLayerListener;

public class HALTesterClassDetection extends HALTesterClass implements HardwareAbstractionLayerListener {
	HardwareAbstractionLayer hal;

	static String equipletName = "EQD";
	static final String baseDir = "generatedOutput/";
	static boolean insertModules = true;
	static boolean translateSteps = true;

	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		if(args.length >= 2) {
			if(args[0].equals("--noInsert")) {
				insertModules = false;
			} else if(args[0].equals("--noTranslate")) {
				translateSteps = false;
			}
			equipletName = args[1];
		} else if(args.length >= 1) {
			equipletName = args[0];
		}
		System.out.println("Inserting equiplet " + equipletName);
		
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "Starting");
		@SuppressWarnings("unused")
		HALTesterClassDetection htc = new HALTesterClassDetection();
		htc = null;
	}
	public HALTesterClassDetection() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException, ParseException {
		hal = new HardwareAbstractionLayer(equipletName, this);

		if(insertModules == true) {
			ModuleIdentifier moduleIdentifierCamera = new ModuleIdentifier("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "3");
			StaticSettings staticSettingsCamera = getStaticSettings(moduleIdentifierCamera);
			DynamicSettings dynamicSettingsCamera = new DynamicSettings();
			dynamicSettingsCamera.mountPointX = 3;
			dynamicSettingsCamera.mountPointY = 16;
			
			ModuleIdentifier moduleIdentifierLens = new ModuleIdentifier("The_Imaging_Source_Europe_GmbH", "cheap_ass_lens", "3");
			StaticSettings staticSettingsLens = getStaticSettings(moduleIdentifierLens);
			DynamicSettings dynamicSettingsLens = new DynamicSettings();
			dynamicSettingsLens.attachedTo = moduleIdentifierCamera;
			
			ModuleIdentifier moduleIdentifierWorkplane = new ModuleIdentifier("HU", "workplane_type_A", "3");
			StaticSettings staticSettingsWorkplane = getStaticSettings(moduleIdentifierWorkplane);
			DynamicSettings dynamicSettingsWorkplane = new DynamicSettings();
			dynamicSettingsWorkplane.mountPointX = 1;
			dynamicSettingsWorkplane.mountPointY = 10;
			
			hal.insertModule(staticSettingsCamera.serialize(), dynamicSettingsCamera.serialize());
			hal.insertModule(staticSettingsLens.serialize(), dynamicSettingsLens.serialize());
			hal.insertModule(staticSettingsWorkplane.serialize(), dynamicSettingsWorkplane.serialize());
		}
		
		// we are done if we are not going to translate hw steps
		if(translateSteps == false) {
			hal.shutdown();
		}
	}
}
