package HAL.testerClasses;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;
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
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

public class HALTesterClassDetection implements HardwareAbstractionLayerListener {
	HardwareAbstractionLayer hal;
	JSONObject criteria1 = new JSONObject();
	JSONObject criteria2 = new JSONObject();
	boolean state = false;

	static String equipletName = "EQD";
	static final String baseDir = "generatedOutput/";
	static boolean insertModules = false;
	static boolean translateSteps = true;

	// camera
	static String moduleA_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\"," +
			"		\"typeNumber\":\"DFK_22AUC03\"," +
			"		\"serialNumber\":\"3\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleA_02 = "\"," +
			"			\"command\":\"roslaunch camera.launch isSimulated:={isSimulated} isShadow:={isShadow} equipletName:={equipletName} manufacturer:={manufacturer} typeNumber:={typeNumber} serialNumber:={serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleA_03 = "\"," +
			"			\"className\":\"HAL.modules.Camera\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleA_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"base\"," +
			"			\"childLink\":\"base\"," +
			"			\"childLinkOffsetX\":-25.01," +
			"			\"childLinkOffsetY\":202.24," +
			"			\"childLinkOffsetZ\":57.19" +
			"		}," +
			"		\"supportedMutations\": [" +
			"		]," +
			"		\"capabilities\":[" +
			"		]" +
			"	}," +
			"	\"properties\":{}," +
			"	\"calibrationData\":[" +
			"	]," +
			"	\"attachedTo\":null," +
			"	\"mountPointX\":3," +
			"	\"mountPointY\":16" +
			"}";
	// lens
	static String moduleB_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\"," +
			"		\"typeNumber\":\"Cheap_ass_lens\"," +
			"		\"serialNumber\":\"3\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{}," +
			"		\"rosSoftware\":null," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleB_02 = "\"," +
			"			\"className\":\"HAL.modules.Lens\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleB_03 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"base\"," +
			"			\"childLink\":\"base\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":22.0" +
			"		}," +
			"		\"supportedMutations\": [" +
			"		]," +
			"		\"capabilities\":[" +
			"		]" +
			"	}," +
			"	\"properties\":{}," +
			"	\"calibrationData\":[" +
			"	]," +
			"	\"attachedTo\":{" +
			"		\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\"," +
			"		\"typeNumber\":\"DFK_22AUC03\"," +
			"		\"serialNumber\":\"3\"" +
			"	}," +
			"	\"mountPointX\":null," +
			"	\"mountPointY\":null" +
			"}";
	// workplane
	static String moduleC_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"workplane_type_A\"," +
			"		\"serialNumber\":\"3\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{" +
			"			\"midPointX\" : 175.0," +
			"			\"midPointY\" : -200.0," +
			"			\"midPointZ\" : 33.33," +
			"			\"topLeftValue\" : \"_WP_TL\"," +
			"			\"topRightValue\" : \"_WP_TR\"," +
			"			\"bottomRightValue\" : \"_WP_BR\"," +
			"			\"workPlaneWidth\" : 80.0," +
			"			\"workPlaneHeight\" : 80.0" +
			"		}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleC_02 = "\"," +
			"			\"command\":\"rosrun part_locator_node part_locator_node {isSimulated} {isshadow} {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleC_03 = "\"," +
			"			\"className\":\"HAL.modules.Workplane\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleC_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"base\"," +
			"			\"childLink\":\"base\"," +
			"			\"childLinkOffsetX\":175.0," +
			"			\"childLinkOffsetY\":-200.0," +
			"			\"childLinkOffsetZ\":33.33" +
			"		}," +
			"		\"supportedMutations\": [" +
			"		]," +
			"		\"capabilities\":[" +
			"		]" +
			"	}," +
			"	\"properties\":{}," +
			"	\"calibrationData\":[" +
			"	]," +
			"	\"attachedTo\":null," +
			"\"mountPointX\":1," +
			"\"mountPointY\":10" +
			"}";

			
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
	public HALTesterClassDetection() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException {
		hal = new HardwareAbstractionLayer(this);

		if(insertModules == true) {
			FileInputStream fis;
			byte[] content;
	
			File workplaneHal = new File(baseDir + "HAL/modules/" + "Workplane.jar");
			fis = new FileInputStream(workplaneHal);
			content = new byte[(int) workplaneHal.length()];
			fis.read(content);
			fis.close();
			String base64WorkplaneHal = new String(Base64.encodeBase64(content));
			
			File penHal = new File(baseDir + "HAL/modules/" + "Pen.jar");
			fis = new FileInputStream(penHal);
			content = new byte[(int) penHal.length()];
			fis.read(content);
			fis.close();
			String base64PenHal = new String(Base64.encodeBase64(content));
			
			File cameraRos = new File(baseDir + "nodes/" + "huniversal_camera.zip");
			fis = new FileInputStream(cameraRos);
			content = new byte[(int) cameraRos.length()];
			fis.read(content);
			fis.close();
			String base64CameraRos = new String(Base64.encodeBase64(content));
			
			File workplaneRos = new File(baseDir + "nodes/" + "workplane.zip");
			fis = new FileInputStream(workplaneRos);
			content = new byte[(int) workplaneRos.length()];
			fis.read(content);
			fis.close();
			String base64WorkplaneRos = new String(Base64.encodeBase64(content));
			
			File cameraGazebo = new File(baseDir + "models/" + "camera.zip");
			fis = new FileInputStream(cameraGazebo);
			content = new byte[(int) cameraGazebo.length()];
			fis.read(content);
			fis.close();
			String base64CameraGazebo = new String(Base64.encodeBase64(content));
			
			File lensGazebo = new File(baseDir + "models/" + "lens.zip");
			fis = new FileInputStream(lensGazebo);
			content = new byte[(int) lensGazebo.length()];
			fis.read(content);
			fis.close();
			String base64LensGazebo = new String(Base64.encodeBase64(content));
			
			File workplaneGazebo = new File(baseDir + "models/" + "workplane.zip");
			fis = new FileInputStream(workplaneGazebo);
			content = new byte[(int) workplaneGazebo.length()];
			fis.read(content);
			fis.close();
			String base64WorkplaneGazebo = new String(Base64.encodeBase64(content));
			
			
			// camera
			String moduleA = moduleA_01 + base64CameraRos + moduleA_02 + base64PenHal + 
					moduleA_03 + base64CameraGazebo + moduleA_04;
			JSONObject a = new JSONObject(new JSONTokener(moduleA));
			
			// lens
			// TODO fix non hal software
			String moduleB = moduleB_01 + "" + moduleB_02 + base64LensGazebo + moduleB_03;
			JSONObject b = new JSONObject(new JSONTokener(moduleB));
			
			// workplane
			String moduleC = moduleC_01 + base64WorkplaneRos + moduleC_02 + base64WorkplaneHal + 
					moduleC_03 + base64WorkplaneGazebo + moduleC_04;
			JSONObject c = new JSONObject(new JSONTokener(moduleC));
			
			
			hal.insertModule(a, a);
			hal.insertModule(b, b);
			hal.insertModule(c, c);
		}
		
		// we are done if we are not going to translate hw steps
		if(translateSteps == false) {
			hal.shutdown();
		}
	}
	
	@Override
	public void onTranslationFinished(String service, JSONObject criteria, ArrayList<HardwareStep> hardwareSteps) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onTranslationFailed(String service, JSONObject criteria) {
		Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, "Translation failed of the following product step:", new Object[]{ service, criteria });
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
		return equipletName;
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
		Logger.log(LogSection.NONE, LogLevel.ERROR, "Execution failed");
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
