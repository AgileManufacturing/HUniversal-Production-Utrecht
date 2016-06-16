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

public class HALTesterClassDummy extends HALTesterClass implements HardwareAbstractionLayerListener {
	HardwareAbstractionLayer hal;

	static String equipletName = "EQ3";
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
		HALTesterClassDummy htc = new HALTesterClassDummy();
		htc = null;
	}
	public HALTesterClassDummy() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException, ParseException {
		hal = new HardwareAbstractionLayer(equipletName, this);

		if(insertModules == true) {
			ModuleIdentifier moduleIdentifierDummyA = new ModuleIdentifier("HU", "dummy_module_type_A", "1");
			StaticSettings staticSettingsDummyA = getStaticSettings(moduleIdentifierDummyA);
			DynamicSettings dynamicSettingsDummyA = new DynamicSettings();
			dynamicSettingsDummyA.mountPointX = 1;
			dynamicSettingsDummyA.mountPointY = 10;
			
			ModuleIdentifier moduleIdentifierDummyB = new ModuleIdentifier("HU", "dummy_module_type_B", "1");
			StaticSettings staticSettingsDummyB = getStaticSettings(moduleIdentifierDummyB);
			DynamicSettings dynamicSettingsDummyB = new DynamicSettings();
			dynamicSettingsDummyB.mountPointX = 3;
			dynamicSettingsDummyB.mountPointY = 4;
			
			staticSettingsDummyA.moduleIdentifier.serialNumber = String.valueOf(serialNumber);
			staticSettingsDummyB.moduleIdentifier.serialNumber = String.valueOf(serialNumber);
			
			hal.insertModule(staticSettingsDummyA.serialize(), dynamicSettingsDummyA.serialize());
			hal.insertModule(staticSettingsDummyB.serialize(), dynamicSettingsDummyB.serialize());
		}
		
		// we are done if we are not going to translate hw steps
		if(translateSteps == false) {
			hal.shutdown();
		}
	}
}
