package HAL.testerClasses;

import generic.Mast;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.exceptions.BlackboardUpdateException;
import HAL.exceptions.InvalidMastModeException;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.listeners.EquipletListener.EquipletCommandStatus;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;

public class HALTesterClass implements HardwareAbstractionLayerListener {
	HardwareAbstractionLayer hal;
	JSONObject criteria1 = new JSONObject();
	
	static String equipletName = "EQ0";
	static final String baseDir = "generatedOutput/";
	static boolean insertModules = true;
	static boolean translateSteps = true;
	static int serialNumber = 1;
	
	// dummy module A
	static String moduleA_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"dummy_module_type_A\"," +
			"		\"serialNumber\":\"1\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleA_02 = "\"," +
			"			\"command\":\"rosrun dummy_module_a dummy_module_a {isSimulated} {isshadow} {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleA_03 = "\"," +
			"			\"className\":\"HAL.modules.DummyModuleA\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleA_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"base\"," +
			"			\"childLink\":\"base\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
			"		}," +
			"		\"supportedMutations\": [" +
			"		]," +
			"		\"capabilities\":[]" +
			"	}," +
			"	\"properties\":{}," +
			"	\"calibrationData\":[" +
			"	]," +
			"	\"attachedTo\":null," +
			"\"mountPointX\":1," +
			"\"mountPointY\":10" +
			"}";
	// dummy module B
	static String moduleB_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"dummy_module_type_B\"," +
			"		\"serialNumber\":\"1\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleB_02 = "\"," +
			"			\"command\":\"rosrun dummy_module_b dummy_module_b {isSimulated} {isshadow} {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleB_03 = "\"," +
			"			\"className\":\"HAL.modules.DummyModuleB\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleB_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"base\"," +
			"			\"childLink\":\"effector\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
			"		}," +
			"		\"supportedMutations\": [" +
			"			\"move\", \"pick\", \"place\"" +
			"		]," +
			"		\"capabilities\":[" + 
			"			{" +
			"				\"name\":\"PickAndPlace\"," +
			"				\"treeNumber\":1," +
			"				\"halSoftware\":{" +
			"					\"buildNumber\":1," +
			"					\"jarFile\": \"";
	static String moduleB_05 = "\"," +
			"					\"className\":\"HAL.capabilities.PickAndPlace\"" +
			"				}," +
			"				\"requiredMutationsTrees\":[" +
			"					{" +
			"						\"treeNumber\":1," +
			"						\"mutations\":[" +
			"							\"move\", \"pick\", \"place\"" +
			"						]" +
			"					}" +
			"				]," +
			"				\"services\":[" +
			"					\"place\"" +
			"				]" +
			"			}" +
			"		]" +
			"	}," +
			"	\"properties\":{}," +
			"	\"calibrationData\":[" +
			"	]," +
			"	\"attachedTo\":null," +
			"\"mountPointX\":3," +
			"\"mountPointY\":4" +
			"}";
	
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
	public HALTesterClass() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException {
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "Starting");
		
		hal = new HardwareAbstractionLayer(equipletName, this);

		if(insertModules == true) {
			FileInputStream fis;
			byte[] content;
	
			File dummyModuleAHal = new File(baseDir + "HAL/modules/" + "DummyModuleA.jar");
			fis = new FileInputStream(dummyModuleAHal);
			content = new byte[(int) dummyModuleAHal.length()];
			fis.read(content);
			fis.close();
			String base64DummyModuleAHal = new String(Base64.encodeBase64(content));
			
			File dummyModuleBHal = new File(baseDir + "HAL/modules/" + "DummyModuleB.jar");
			fis = new FileInputStream(dummyModuleBHal);
			content = new byte[(int) dummyModuleBHal.length()];
			fis.read(content);
			fis.close();
			String base64DummyModuleBHal = new String(Base64.encodeBase64(content));
			
			File dummyModuleARos = new File(baseDir + "nodes/" + "_testing.zip");
			fis = new FileInputStream(dummyModuleARos);
			content = new byte[(int) dummyModuleARos.length()];
			fis.read(content);
			fis.close();
			String base64DummyModuleARos = new String(Base64.encodeBase64(content));
			
			File dummyModuleBRos = new File(baseDir + "nodes/" + "_testing.zip");
			fis = new FileInputStream(dummyModuleBRos);
			content = new byte[(int) dummyModuleBRos.length()];
			fis.read(content);
			fis.close();
			String base64DummyModuleBRos = new String(Base64.encodeBase64(content));
			
			File dummyModuleAGazebo = new File(baseDir + "models/" + "workplane.zip");
			fis = new FileInputStream(dummyModuleAGazebo);
			content = new byte[(int) dummyModuleAGazebo.length()];
			fis.read(content);
			fis.close();
			String base64DummyModuleAGazebo = new String(Base64.encodeBase64(content));
			
			File dummyModuleBGazebo = new File(baseDir + "models/" + "sixAxis.zip");
			fis = new FileInputStream(dummyModuleBGazebo);
			content = new byte[(int) dummyModuleBGazebo.length()];
			fis.read(content);
			fis.close();
			String base64DummyModuleBGazebo = new String(Base64.encodeBase64(content));
			
			File pickAndPlaceHal = new File(baseDir + "HAL/capabilities/" + "PickAndPlace.jar");
			fis = new FileInputStream(pickAndPlaceHal);
			content = new byte[(int) pickAndPlaceHal.length()];
			fis.read(content);
			fis.close();
			String base64PickAndPlace = new String(Base64.encodeBase64(content));
			
			// dummy module A
			String moduleA = moduleA_01 + base64DummyModuleARos + moduleA_02 + base64DummyModuleAHal + 
					moduleA_03 + base64DummyModuleAGazebo + moduleA_04;
			JSONObject a = new JSONObject(new JSONTokener(moduleA));
			
			// dummy module B
			String moduleB = moduleB_01 + base64DummyModuleBRos + moduleB_02 + base64DummyModuleBHal + 
					moduleB_03 + base64DummyModuleBGazebo + moduleB_04 + base64PickAndPlace + moduleB_05;
			JSONObject b = new JSONObject(new JSONTokener(moduleB));
			
			a.getJSONObject("moduleIdentifier").put("serialNumber", String.valueOf(serialNumber));
			b.getJSONObject("moduleIdentifier").put("serialNumber", String.valueOf(serialNumber));
			
			hal.insertModule(a, a);
			hal.insertModule(b, b);
		}
		
		// we are done if we are not going to translate hw steps
		if(translateSteps == false) {
			hal.shutdown();
		} else {
			// wait for ros node to come online
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			hal.changeState(Mast.State.SAFE);
		}
	}
	
	@Override
	public void onTranslationFinished(String service, JSONObject criteria, ArrayList<HardwareStep> hardwareSteps) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onTranslationFailed(String service, JSONObject criteria) {
		Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, equipletName + " Translation failed of the following product step:", new Object[]{ service, criteria });
	}

	@Override
	public void onProcessStatusChanged(Module module, HardwareStep hardwareStep) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The status of " + hardwareStep + " (being processed by module " + module + ") has changed to " + hardwareStep.getStatus());
	}

	@Override
	public void onModuleStateChanged(Module module, Mast.State state) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of module " + module + " has changed to " + state);
	}

	@Override
	public void onModuleModeChanged(Module module, Mast.Mode mode) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The mode of module " + module + " has changed to " + mode);
	}

	@Override
	public void onExecutionFinished() {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Execution finished");
		hal.translateProductStep("place", criteria1);
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
		try{
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
	
	
			criteria1.put("target", target1);
			criteria1.put("subjects", subjects1);
		} catch (JSONException ex) {
			ex.printStackTrace();
		}
		
		hal.translateProductStep("place", criteria1);
	}

	@Override
	public void onExecutionFailed() {
		Logger.log(LogSection.NONE, LogLevel.ERROR, equipletName + " Execution failed");
	}

	@Override
	public void onEquipletCommandStatusChanged(EquipletCommandStatus status) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Reloading has: " + status);
	}
}
