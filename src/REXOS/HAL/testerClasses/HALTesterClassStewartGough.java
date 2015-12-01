package HAL.testerClasses;

import java.io.IOException;
import java.text.ParseException;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

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

public class HALTesterClassStewartGough extends HALTesterClass implements HardwareAbstractionLayerListener {
	JSONObject criteria1 = new JSONObject();
	JSONObject criteria2 = new JSONObject();
	boolean state = false;

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
		HALTesterClassStewartGough htc = new HALTesterClassStewartGough();
		htc = null;
	}
	public HALTesterClassStewartGough() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException, ParseException {
		hal = new HardwareAbstractionLayer(equipletName, this);

		if(insertModules == true) {
			ModuleIdentifier moduleIdentifierStewartGough = new ModuleIdentifier("HU", "stewart_gough_type_A", "1");
			StaticSettings staticSettingsStewartGough = getStaticSettings(moduleIdentifierStewartGough);
			DynamicSettings dynamicSettingsStewartGough = new DynamicSettings();
			dynamicSettingsStewartGough.mountPointX = 3;
			dynamicSettingsStewartGough.mountPointY = 2;
			
			ModuleIdentifier moduleIdentifierGripper = new ModuleIdentifier("HU", "gripper_type_A", "2");
			StaticSettings staticSettingsGripper = getStaticSettings(moduleIdentifierGripper);
			DynamicSettings dynamicSettingsGripper = new DynamicSettings();
			dynamicSettingsGripper.attachedTo = moduleIdentifierStewartGough;
			
			ModuleIdentifier moduleIdentifierCamera = new ModuleIdentifier("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "2");
			StaticSettings staticSettingsCamera = getStaticSettings(moduleIdentifierCamera);
			DynamicSettings dynamicSettingsCamera = new DynamicSettings();
			dynamicSettingsCamera.mountPointX = 3;
			dynamicSettingsCamera.mountPointY = 16;
			
			ModuleIdentifier moduleIdentifierLens = new ModuleIdentifier("The_Imaging_Source_Europe_GmbH", "cheap_ass_lens", "2");
			StaticSettings staticSettingsLens = getStaticSettings(moduleIdentifierLens);
			DynamicSettings dynamicSettingsLens = new DynamicSettings();
			dynamicSettingsLens.attachedTo = moduleIdentifierCamera;
			
			ModuleIdentifier moduleIdentifierWorkplane = new ModuleIdentifier("HU", "workplane_type_A", "2");
			StaticSettings staticSettingsWorkplane = getStaticSettings(moduleIdentifierWorkplane);
			DynamicSettings dynamicSettingsWorkplane = new DynamicSettings();
			dynamicSettingsWorkplane.mountPointX = 1;
			dynamicSettingsWorkplane.mountPointY = 10;
			
			hal.insertModule(staticSettingsStewartGough.serialize(), dynamicSettingsStewartGough.serialize());
			hal.insertModule(staticSettingsGripper.serialize(), dynamicSettingsGripper.serialize());
			hal.insertModule(staticSettingsCamera.serialize(), dynamicSettingsCamera.serialize());
			hal.insertModule(staticSettingsLens.serialize(), dynamicSettingsLens.serialize());
			hal.insertModule(staticSettingsWorkplane.serialize(), dynamicSettingsWorkplane.serialize());
		}
		
		if(translateSteps == true) {
			JSONObject target1 = new JSONObject();
			JSONObject targetMove1 = new JSONObject();
			targetMove1.put("x", 5.5);
			targetMove1.put("y", -5.5);
			targetMove1.put("z", 13.8);
			targetMove1.put("maxAcceleration", 50.0);
			JSONObject targetMove1Approach = new JSONObject();
			targetMove1Approach.put("x", 0);
			targetMove1Approach.put("y", 0);
			targetMove1Approach.put("z", 20);
			targetMove1.put("approach", targetMove1Approach);
			JSONObject targetRotate1 = new JSONObject();
			targetRotate1.put("x", 0);
			targetRotate1.put("y", 0);
			targetRotate1.put("z", 0);
			JSONObject targetRotate1Approach = new JSONObject();
			targetRotate1Approach.put("x", 0);
			targetRotate1Approach.put("y", 0);
			targetRotate1Approach.put("z", 0);
			targetRotate1.put("approach", targetRotate1Approach);
			target1.put("move", targetMove1);
			target1.put("rotate", targetRotate1);
			target1.put("identifier", "GC4x4MB_1");
	
			JSONArray subjects1 = new JSONArray();
			JSONObject subject1 = new JSONObject();
			JSONObject subjectMove1 = new JSONObject();
			subjectMove1.put("x", 5.5);
			subjectMove1.put("y", -5.5);
			subjectMove1.put("z", 13.8);
			subjectMove1.put("maxAcceleration", 50.0);
			JSONObject subjectRotate1 = new JSONObject();
			subjectRotate1.put("x", 0);
			subjectRotate1.put("y", 0);
			subjectRotate1.put("z", 0);
			JSONObject subjectMove1Approach = new JSONObject();
			subjectMove1Approach.put("x", 0);
			subjectMove1Approach.put("y", 0);
			subjectMove1Approach.put("z", 20);
			JSONObject subjectRotate1Approach = new JSONObject();
			subjectRotate1Approach.put("x", 0);
			subjectRotate1Approach.put("y", 0);
			subjectRotate1Approach.put("z", 0);
			subjectMove1.put("approach", subjectMove1Approach);
			subjectRotate1.put("approach", subjectRotate1Approach);
			subject1.put("move", subjectMove1);
			subject1.put("rotate", subjectRotate1);
			subject1.put("identifier", "GC4x4MB_6");
			subjects1.put(subject1);
	
			JSONObject target2 = new JSONObject();
			JSONObject targetMove2 = new JSONObject();
			targetMove2.put("x", 5.5);
			targetMove2.put("y", -5.5);
			targetMove2.put("z", 13.8);
			targetMove2.put("maxAcceleration", 50.0);
			JSONObject targetMove2Approach = new JSONObject();
			targetMove2Approach.put("x", 0);
			targetMove2Approach.put("y", 0);
			targetMove2Approach.put("z", 20);
			targetMove2.put("approach", targetMove2Approach);
			JSONObject targetRotate2 = new JSONObject();
			targetRotate2.put("x", 0);
			targetRotate2.put("y", 0);
			targetRotate2.put("z", 0);
			JSONObject targetRotate2Approach = new JSONObject();
			targetRotate2Approach.put("x", 0);
			targetRotate2Approach.put("y", 0);
			targetRotate2Approach.put("z", 0);
			targetRotate2.put("approach", targetRotate2Approach);
			target2.put("move", targetMove2);
			target2.put("rotate", targetRotate2);
			target2.put("identifier", "GC4x4MB_6");
	
			JSONArray subjects2 = new JSONArray();
			JSONObject subject2 = new JSONObject();
			JSONObject subjectMove2 = new JSONObject();
			subjectMove2.put("x", 5.5);
			subjectMove2.put("y", -5.5);
			subjectMove2.put("z", 13.8);
			subjectMove2.put("maxAcceleration", 50.0);
			JSONObject subjectRotate2 = new JSONObject();
			subjectRotate2.put("x", 0);
			subjectRotate2.put("y", 0);
			subjectRotate2.put("z", 0);
			JSONObject subjectMove2Approach = new JSONObject();
			subjectMove2Approach.put("x", 0);
			subjectMove2Approach.put("y", 0);
			subjectMove2Approach.put("z", 20);
			JSONObject subjectRotate2Approach = new JSONObject();
			subjectRotate2Approach.put("x", 0);
			subjectRotate2Approach.put("y", 0);
			subjectRotate2Approach.put("z", 0);
			subjectMove2.put("approach", subjectMove2Approach);
			subjectRotate2.put("approach", subjectRotate2Approach);
			subject2.put("move", subjectMove2);
			subject2.put("rotate", subjectRotate2);
			subject2.put("identifier", "GC4x4MB_1");
			subjects2.put(subject2);
	
			criteria1.put("target", target1);
			criteria1.put("subjects", subjects1);
	
			criteria2.put("target", target2);
			criteria2.put("subjects", subjects2);
	
			for(int i = 1; i<100; i++){
				hal.translateProductStep("place", criteria1);
				hal.translateProductStep("place", criteria2);
			}
		}
		
		// we are done if we are not going to translate hw steps
		if(translateSteps == false) {
			hal.shutdown();
		}
	}
}
