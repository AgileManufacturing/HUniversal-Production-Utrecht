package HAL.testerClasses;

import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;

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
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

public class HALTesterClassDraw extends HALTesterClass implements HardwareAbstractionLayerListener {
	HardwareAbstractionLayer hal;
	JSONObject criteria1 = new JSONObject();
	JSONObject criteria2 = new JSONObject();
	boolean state = false;

	static String equipletName = "EQ2";
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
		HALTesterClassDraw htc = new HALTesterClassDraw();
		htc = null;
	}
	public HALTesterClassDraw() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException, ParseException {
		hal = new HardwareAbstractionLayer(equipletName, this);

		if(insertModules == true) {
			ModuleIdentifier moduleIdentifierDeltaRobot = new ModuleIdentifier("HU", "delta_robot_type_B", "1");
			StaticSettings staticSettingsDeltaRobot = getStaticSettings(moduleIdentifierDeltaRobot);
			DynamicSettings dynamicSettingsDeltaRobot = new DynamicSettings();
			dynamicSettingsDeltaRobot.mountPointX = 3;
			dynamicSettingsDeltaRobot.mountPointY = 1;
			
			ModuleIdentifier moduleIdentifierPen = new ModuleIdentifier("HU", "pen_type_A", "1");
			StaticSettings staticSettingsPen = getStaticSettings(moduleIdentifierPen);
			DynamicSettings dynamicSettingsPen = new DynamicSettings();
			dynamicSettingsPen.attachedTo = moduleIdentifierDeltaRobot;
			
			ModuleIdentifier moduleIdentifierCamera = new ModuleIdentifier("The_Imaging_Source_Europe_GmbH", "DFK_22AUC03", "1");
			StaticSettings staticSettingsCamera = getStaticSettings(moduleIdentifierCamera);
			DynamicSettings dynamicSettingsCamera = new DynamicSettings();
			dynamicSettingsCamera.mountPointX = 3;
			dynamicSettingsCamera.mountPointY = 16;
			
			ModuleIdentifier moduleIdentifierLens = new ModuleIdentifier("The_Imaging_Source_Europe_GmbH", "cheap_ass_lens", "1");
			StaticSettings staticSettingsLens = getStaticSettings(moduleIdentifierLens);
			DynamicSettings dynamicSettingsLens = new DynamicSettings();
			dynamicSettingsLens.attachedTo = moduleIdentifierCamera;
			
			ModuleIdentifier moduleIdentifierWorkplane = new ModuleIdentifier("HU", "workplane_type_A", "1");
			StaticSettings staticSettingsWorkplane = getStaticSettings(moduleIdentifierWorkplane);
			DynamicSettings dynamicSettingsWorkplane = new DynamicSettings();
			dynamicSettingsWorkplane.mountPointX = 1;
			dynamicSettingsWorkplane.mountPointY = 10;
			
			hal.insertModule(staticSettingsDeltaRobot.serialize(), dynamicSettingsDeltaRobot.serialize());
			hal.insertModule(staticSettingsPen.serialize(), dynamicSettingsPen.serialize());
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
			JSONObject targetMove1Approach = new JSONObject();
			targetMove1Approach.put("x", 0);
			targetMove1Approach.put("y", 0);
			targetMove1Approach.put("z", 20);
			targetMove1.put("approach", targetMove1Approach);
			target1.put("move", targetMove1);
			target1.put("identifier", "GC4x4MB_1");
	
			JSONArray subjects1 = new JSONArray();
			JSONObject subject1 = new JSONObject();
			JSONObject subjectMove1 = new JSONObject();
			subjectMove1.put("x", 5.5);
			subjectMove1.put("y", -5.5);
			subjectMove1.put("z", 13.8);
			JSONObject subjectMove1Approach = new JSONObject();
			subjectMove1Approach.put("x", 0);
			subjectMove1Approach.put("y", 0);
			subjectMove1Approach.put("z", 20);
			subjectMove1.put("approach", subjectMove1Approach);
			subject1.put("move", subjectMove1);
			subject1.put("identifier", "GC4x4MB_6");
			subjects1.put(subject1);
	
			JSONObject target2 = new JSONObject();
			JSONObject targetMove2 = new JSONObject();
			targetMove2.put("x", 5.5);
			targetMove2.put("y", -5.5);
			targetMove2.put("z", 13.8);
			JSONObject targetMove2Approach = new JSONObject();
			targetMove2Approach.put("x", 0);
			targetMove2Approach.put("y", 0);
			targetMove2Approach.put("z", 20);
			targetMove2.put("approach", targetMove2Approach);
			target2.put("move", targetMove2);
			target2.put("identifier", "GC4x4MB_6");
	
			JSONArray subjects2 = new JSONArray();
			JSONObject subject2 = new JSONObject();
			JSONObject subjectMove2 = new JSONObject();
			subjectMove2.put("x", 5.5);
			subjectMove2.put("y", -5.5);
			subjectMove2.put("z", 13.8);
			JSONObject subjectMove2Approach = new JSONObject();
			subjectMove2Approach.put("x", 0);
			subjectMove2Approach.put("y", 0);
			subjectMove2Approach.put("z", 20);
			subjectMove2.put("approach", subjectMove2Approach);
			subject2.put("move", subjectMove2);
			subject2.put("identifier", "GC4x4MB_1");
			subjects2.put(subject2);
	
			criteria1.put("target", target1);
			criteria1.put("subjects", subjects1);
	
			criteria2.put("target", target2);
			criteria2.put("subjects", subjects2);
	
			hal.translateProductStep("place", criteria1);
		}
		
		// we are done if we are not going to translate hw steps
		if(translateSteps == false) {
			hal.shutdown();
		}
	}
	
	@Override
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareSteps) {
		super.onTranslationFinished(productStep, hardwareSteps);
		hal.executeHardwareSteps(hardwareSteps);
	}
}