package HAL.testerClasses;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONObject;
import org.json.JSONTokener;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.BlackboardHandler;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.dataTypes.ModuleIdentifier;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

public class HALTesterClass implements HardwareAbstractionLayerListener {
	static HALTesterClass htc = new HALTesterClass();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	static BlackboardHandler blackboardUpdated;

	static final String baseDir = "jars/";
	
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
			"			\"command\":\"rosrun dummy_module_a dummy_module_a {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
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
			"			\"parentLink\":\"baseLink\"," +
			"			\"childLink\":\"otherLink\"" +
			"		}," +
			"		\"supportedMutations\": [" +
			"		]," +
			"		\"capabilities\":[]" +
			"	}," +
			"	\"properties\":{}," +
			"	\"calibrationData\":[" +
			"	]," +
			"	\"attachedTo\":null," +
			"\"mountPointX\":3," +
			"\"mountPointY\":2" +
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
			"			\"command\":\"rosrun dummy_module_b dummy_module_b {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
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
			"			\"parentLink\":\"baseLink\"," +
			"			\"childLink\":\"otherLink\"" +
			"		}," +
			"		\"supportedMutations\": [" +
			"		]," +
			"		\"capabilities\":[]" +
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
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "Starting");
		
		hal = new HardwareAbstractionLayer(htc);

		FileInputStream fis;
		byte[] content;

		File dummyModuleAHal = new File(baseDir + "DummyModuleA.jar");
		fis = new FileInputStream(dummyModuleAHal);
		content = new byte[(int) dummyModuleAHal.length()];
		fis.read(content);
		fis.close();
		String base64DummyModuleAHal = new String(Base64.encodeBase64(content));
		
		File dummyModuleBHal = new File(baseDir + "DummyModuleB.jar");
		fis = new FileInputStream(dummyModuleBHal);
		content = new byte[(int) dummyModuleBHal.length()];
		fis.read(content);
		fis.close();
		String base64DummyModuleBHal = new String(Base64.encodeBase64(content));
		
		File dummyModuleARos = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(dummyModuleARos);
		content = new byte[(int) dummyModuleARos.length()];
		fis.read(content);
		fis.close();
		String base64DummyModuleARos = new String(Base64.encodeBase64(content));
		
		File dummyModuleBRos = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(dummyModuleBRos);
		content = new byte[(int) dummyModuleBRos.length()];
		fis.read(content);
		fis.close();
		String base64DummyModuleBRos = new String(Base64.encodeBase64(content));
		
		File dummyModuleAGazebo = new File(baseDir + "model.zip");
		fis = new FileInputStream(dummyModuleAGazebo);
		content = new byte[(int) dummyModuleAGazebo.length()];
		fis.read(content);
		fis.close();
		String base64DummyModuleAGazebo = new String(Base64.encodeBase64(content));
		
		File dummyModuleBGazebo = new File(baseDir + "model.zip");
		fis = new FileInputStream(dummyModuleBGazebo);
		content = new byte[(int) dummyModuleBGazebo.length()];
		fis.read(content);
		fis.close();
		String base64DummyModuleBGazebo = new String(Base64.encodeBase64(content));
		
		// dummy module A
		String moduleA = moduleA_01 + base64DummyModuleARos + moduleA_02 + base64DummyModuleAHal + 
				moduleA_03 + base64DummyModuleAGazebo + moduleA_04;
		JSONObject a = new JSONObject(new JSONTokener(moduleA));
		
		// dummy module B
		String moduleB = moduleB_01 + base64DummyModuleBRos + moduleB_02 + base64DummyModuleBHal + 
				moduleB_03 + base64DummyModuleBGazebo + moduleB_04;
		JSONObject b = new JSONObject(new JSONTokener(moduleB));
		
		
		hal.insertModule(a, a);
		hal.insertModule(b, b);
		
		
		
		//hal.translateProductStep(new ProductStep(1, criteria, new Service("place")));
		// Sending reloadEquiplet command to Blackboard
		// UNTESTED W.I.P (Lars Veenendaal)]
		hal.deleteModule(new ModuleIdentifier("HU", "dummy_module_type_A", "1"));
		hal.deleteModule(new ModuleIdentifier("HU", "dummy_module_type_B", "1"));
		hal.sendReloadEquiplet();
		/*Service service = new Service("PickAndPlace");
		ProductStep productStep = new ProductStep(0, null, service);
		hal.translateProductStep(productStep);*/
		

	}
	
	@Override
	public void onTranslationFinished(String service, JSONObject criteria, ArrayList<HardwareStep> hardwareStep) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hardwareSteps.addAll(hardwareStep);// = hardwareStep;
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onTranslationFailed(String service, JSONObject criteria) {
		Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, "Translation failed of the following product step:", new Object[] { service, criteria });
	}

	@Override
	public void onProcessStatusChanged(HardwareStepStatus status, 
			Module module, HardwareStep hardwareStep) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The status of " + hardwareStep + " (being processed by module " + module + ") has changed to " + status);
	}

	@Override
	public void onModuleStateChanged(String state, Module module) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of module " + module + " has changed to " + state);
	}

	@Override
	public void onModuleModeChanged(String mode, Module module) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The mode of module " + module + " has changed to " + mode);
	}

	@Override
	public String getEquipletName() {
		// TODO hardcoded!!!!!!
		
		return "EQ2";
	}

	@Override
	public void onExecutionFinished() {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Execution finished");
	}

	@Override
	public void onEquipletStateChanged(String state) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of equiplet " + getEquipletName() + " has changed to " + state);
	}

	@Override
	public void onEquipletModeChanged(String mode) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The mode of equiplet " + getEquipletName() + " has changed to " + mode);
	}

	@Override
	public void onExecutionFailed() {
		// TODO Auto-generated method stub
		
	}

	/**
	 * [onReloadEquiplet -Test function W.I.P (Lars Veenendaal)]
	 * @param state [description]
	 */
	@Override
	public void onReloadEquiplet(String state){
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Reloading has: " + state);

	}
}
