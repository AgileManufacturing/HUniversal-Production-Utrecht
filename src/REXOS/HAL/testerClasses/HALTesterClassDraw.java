package HAL.testerClasses;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONArray;
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

public class HALTesterClassDraw implements HardwareAbstractionLayerListener {
	static HALTesterClassDraw htc = new HALTesterClassDraw();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	static BlackboardHandler blackboardUpdated;
	
	static final String baseDir = "jars/";
	
	// delta robot
	static String moduleA_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"delta_robot_type_B\"," +
			"		\"serialNumber\":\"1\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{" +
			"			\"midPointX\" : 75.0," +
			"			\"midPointY\" : -200.0," +
			"			\"midPointZ\" : -35.3," +
			"			\"deltaRobotMeasures\" : {" +
			"				\"baseRadius\" : 101.3," +
			"				\"hipLength\" : 100.0," +
			"				\"effectorRadius\" : 46.19," +
			"				\"ankleLength\" : 300.0," +
			"				\"hipAnleMaxAngleDegrees\" : 22.0," +
			"				\"motorFromZeroToTopAngleDegrees\" : 20.0," +
			"				\"boundaryBoxMinX\" : -200.0," +
			"				\"boundaryBoxMaxX\" : 200.0," +
			"				\"boundaryBoxMinY\" : -200.0," +
			"				\"boundaryBoxMaxY\" : 200.0," +
			"				\"boundaryBoxMinZ\" : -380.0," +
			"				\"boundaryBoxMaxZ\" : -180.0" +
			"			}," +
			"			\"calibrationBigStepFactor\" : 20," +
			"			\"stepperMotorProperties\" : {" +
			"				\"motorMinAngleDegrees\" : -18.0," +
			"				\"motorMaxAngleDegrees\" : 90.0," +
			"				\"microStepAngleDegrees\" : 0.036," +
			"				\"minAccelerationDegrees\" : 36," +
			"				\"maxAccelerationDegrees\" : 36000," +
			"				\"minSpeedDegrees\" : 0.036," +
			"				\"maxSpeedDegrees\" : 18000" +
			"			}" +
			"		}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleA_02 = "\"," +
			"			\"command\":\"rosrun delta_robot_node delta_robot_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleA_03 = "\"," +
			"			\"className\":\"HAL.modules.DeltaRobot\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleA_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"baseLink\"," +
			"			\"childLink\":\"otherLink\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
			"		}," +
			"		\"supportedMutations\": [" +
			"			\"move\"" +
			"		]," +
			"		\"capabilities\":[" +
			"			{" +
			"				\"name\":\"Draw\"," +
			"				\"treeNumber\":1," +
			"				\"halSoftware\":{" +
			"					\"buildNumber\":1," +
			"					\"jarFile\": \"";
	static String moduleA_05 = "\"," +
			"					\"className\":\"HAL.capabilities.Draw\"" +
			"				}," +
			"				\"requiredMutationsTrees\":[" +
			"					{" +
			"						\"treeNumber\":1," +
			"						\"mutations\":[" +
			"							\"move\", \"draw\"" +
			"						]" +
			"					}" +
			"				]," +
			"				\"services\":[" +
			"					\"draw\"" +
			"				]" +
			"			}," +
			"			{" +
			"				\"name\":\"PickAndPlace\"," +
			"				\"treeNumber\":1," +
			"				\"halSoftware\":{" +
			"					\"buildNumber\":1," +
			"					\"jarFile\": \"";
	static String moduleA_06 = "\"," +
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
			"	\"properties\":{" +
			"		\"modbusIp\" : \"192.168.0.22\"," +
			"		\"modbusPort\" : 502" +
			"	}," +
			"	\"calibrationData\":[" +
			"		{" +
			"			\"date\":\"2014-01-01 00:00:00\"," +
			"			\"data\":{}," +
			"			\"moduleSet\":[" +
			"				{" +
			"					\"manufacturer\":\"manA\"," +
			"					\"typeNumber\":\"typeA\"," +
			"					\"serialNumber\":\"serA\"" +
			"				}" +
			"			]" +
			"		}" +
			"	]," +
			"	\"attachedTo\":null," +
			"\"mountPointX\":3," +
			"\"mountPointY\":2" +
			"}";
	// gripper
	static String moduleB_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"gripper_type_A\"," +
			"		\"serialNumber\":\"1\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{" +
			"			\"modbusAddress\" : 8001," +
			"			\"modbusDevicePin\" : 0," +
			" 			\"gripperSize\" : 46.54," +
			"			\"gripperEnabledMaxSeconds\" : 60," +
			"			\"gripperEnabledWarningSeconds\" : 50," +
			"			\"gripperEnabledCooldownSeconds\" : 180," +
			"			\"watchdogInterval\" : 100" +
			"		}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleB_02 = "\"," +
			"			\"command\":\"rosrun gripper_node gripper_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleB_03 = "\"," +
			"			\"className\":\"HAL.modules.Gripper\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleB_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"baseLink\"," +
			"			\"childLink\":\"otherLink\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
			"		}," +
			"		\"supportedMutations\": [" +
			"			\"pick\", \"place\"" +
			"		]," +
			"		\"capabilities\":[" +
			"		]" +
			"	}," +
			"	\"properties\":{}," +
			"	\"calibrationData\":[" +
			"	]," +
			"	\"attachedTo\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"delta_robot_type_B\"," +
			"		\"serialNumber\":\"1\"" +
			"	}," +
			"	\"mountPointX\":null," +
			"	\"mountPointY\":null" +
			"}";
	// camera
	static String moduleC_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\"," +
			"		\"typeNumber\":\"DFK_22AUC03\"," +
			"		\"serialNumber\":\"1\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleC_02 = "\"," +
			"			\"command\":\"roslaunch camera.launch equipletName:={equipletName} manufacturer:={manufacturer} typeNumber:={typeNumber} serialNumber:={serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleC_03 = "\"," +
			"			\"className\":\"HAL.modules.Camera\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleC_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"baseLink\"," +
			"			\"childLink\":\"otherLink\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
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
	static String moduleD_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\"," +
			"		\"typeNumber\":\"Cheap_ass_lens\"," +
			"		\"serialNumber\":\"1\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{}," +
			"		\"rosSoftware\":null," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleD_02 = "\"," +
			"			\"className\":\"HAL.modules.Lens\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleD_03 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"baseLink\"," +
			"			\"childLink\":\"otherLink\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
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
			"		\"serialNumber\":\"1\"" +
			"	}," +
			"	\"mountPointX\":null," +
			"	\"mountPointY\":null" +
			"}";
	// workplane
	static String moduleE_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"workplane_type_A\"," +
			"		\"serialNumber\":\"1\"," +
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
	static String moduleE_02 = "\"," +
			"			\"command\":\"rosrun part_locator_node part_locator_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleE_03 = "\"," +
			"			\"className\":\"HAL.modules.Workplane\"" +
			"		}," +
			"		\"gazeboModel\":{" +
			"			\"buildNumber\":1," +
			"			\"zipFile\": \"";
	static String moduleE_04 = "\"," +
			"			\"sdfFilename\":\"model.sdf\"," +
			"			\"parentLink\":\"baseLink\"," +
			"			\"childLink\":\"otherLink\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
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
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "Starting");
		
		hal = new HardwareAbstractionLayer(htc);

		FileInputStream fis;
		byte[] content;

		File deltaRobotHal = new File(baseDir + "DeltaRobot.jar");
		fis = new FileInputStream(deltaRobotHal);
		content = new byte[(int) deltaRobotHal.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotHal = new String(Base64.encodeBase64(content));
		
		File workplaneHal = new File(baseDir + "Workplane.jar");
		fis = new FileInputStream(workplaneHal);
		content = new byte[(int) workplaneHal.length()];
		fis.read(content);
		fis.close();
		String base64WorkplaneHal = new String(Base64.encodeBase64(content));
		
		File penHal = new File(baseDir + "Pen.jar");
		fis = new FileInputStream(penHal);
		content = new byte[(int) penHal.length()];
		fis.read(content);
		fis.close();
		String base64PenHal = new String(Base64.encodeBase64(content));
		
		File gripperHal = new File(baseDir + "Gripper.jar");
		fis = new FileInputStream(gripperHal);
		content = new byte[(int) gripperHal.length()];
		fis.read(content);
		fis.close();
		String base64GripperHal = new String(Base64.encodeBase64(content));
		
		File deltaRobotRos = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(deltaRobotRos);
		content = new byte[(int) deltaRobotRos.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotRos = new String(Base64.encodeBase64(content));
		
		File gripperRos = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(gripperRos);
		content = new byte[(int) gripperRos.length()];
		fis.read(content);
		fis.close();
		String base64GripperRos = new String(Base64.encodeBase64(content));
		
		File cameraRos = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(cameraRos);
		content = new byte[(int) cameraRos.length()];
		fis.read(content);
		fis.close();
		String base64CameraRos = new String(Base64.encodeBase64(content));
		
		File workplaneRos = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(workplaneRos);
		content = new byte[(int) workplaneRos.length()];
		fis.read(content);
		fis.close();
		String base64WorkplaneRos = new String(Base64.encodeBase64(content));
		
		File deltaRobotGazebo = new File(baseDir + "model.zip");
		fis = new FileInputStream(deltaRobotGazebo);
		content = new byte[(int) deltaRobotGazebo.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotGazebo = new String(Base64.encodeBase64(content));
		
		File gripperGazebo = new File(baseDir + "model.zip");
		fis = new FileInputStream(gripperGazebo);
		content = new byte[(int) gripperGazebo.length()];
		fis.read(content);
		fis.close();
		String base64GripperGazebo = new String(Base64.encodeBase64(content));
		
		File cameraGazebo = new File(baseDir + "model.zip");
		fis = new FileInputStream(cameraGazebo);
		content = new byte[(int) cameraGazebo.length()];
		fis.read(content);
		fis.close();
		String base64CameraGazebo = new String(Base64.encodeBase64(content));
		
		File lenstGazebo = new File(baseDir + "model.zip");
		fis = new FileInputStream(lenstGazebo);
		content = new byte[(int) lenstGazebo.length()];
		fis.read(content);
		fis.close();
		String base64LensGazebo = new String(Base64.encodeBase64(content));
		
		File workplaneGazebo = new File(baseDir + "model.zip");
		fis = new FileInputStream(workplaneGazebo);
		content = new byte[(int) workplaneGazebo.length()];
		fis.read(content);
		fis.close();
		String base64WorkplaneGazebo = new String(Base64.encodeBase64(content));
		
		
		File drawHal = new File(baseDir + "Draw.jar");
		fis = new FileInputStream(drawHal);
		content = new byte[(int) drawHal.length()];
		fis.read(content);
		fis.close();
		String base64Draw = new String(Base64.encodeBase64(content));
		
		File pickAndPlaceHal = new File(baseDir + "PickAndPlace.jar");
		fis = new FileInputStream(pickAndPlaceHal);
		content = new byte[(int) pickAndPlaceHal.length()];
		fis.read(content);
		fis.close();
		String base64PickAndPlace = new String(Base64.encodeBase64(content));
		
		
		// deltarobot
		String moduleA = moduleA_01 + base64DeltaRobotRos + moduleA_02 + base64DeltaRobotHal + 
				moduleA_03 + base64DeltaRobotGazebo + moduleA_04 + base64Draw + moduleA_05 + base64PickAndPlace + moduleA_06; 
		JSONObject a = new JSONObject(new JSONTokener(moduleA));
		hal.insertModule(a, a);
		
		// gripper
		String moduleB = moduleB_01 + base64GripperRos + moduleB_02 + base64GripperHal + 
				moduleB_03 + base64GripperGazebo + moduleB_04; 
		JSONObject b = new JSONObject(new JSONTokener(moduleB));
		hal.insertModule(b, b);
		
		// camera
		String moduleC = moduleC_01 + base64CameraRos + moduleC_02 + base64PenHal + 
				moduleC_03 + base64CameraGazebo + moduleC_04;
		JSONObject c = new JSONObject(new JSONTokener(moduleC));
		hal.insertModule(c, c);
		
		// lens
		// TODO fix non hal software
		String moduleD = moduleD_01 + "" + moduleD_02 + base64LensGazebo + moduleD_03;
		JSONObject d = new JSONObject(new JSONTokener(moduleD));
		hal.insertModule(d, d);
		
		// workplane
		String moduleE = moduleE_01 + base64WorkplaneRos + moduleE_02 + base64WorkplaneHal + 
				moduleE_03 + base64WorkplaneGazebo + moduleE_04;
		JSONObject e = new JSONObject(new JSONTokener(moduleE));
		hal.insertModule(e, e);
		
		
		JSONObject criteria = new JSONObject();
		JSONObject target = new JSONObject();
		JSONObject targetMove = new JSONObject();
		targetMove.put("x", -2.0);
		targetMove.put("y", 1.0);
		targetMove.put("z", 15.0);
		target.put("move",targetMove);
		target.put("identifier", "GC4x4MB_1");
		
		JSONArray subjects = new JSONArray();
		JSONObject subject = new JSONObject();
		JSONObject subjectMove = new JSONObject();
		subjectMove.put("x", -3.0);
		subjectMove.put("y", 3.0);
		subjectMove.put("z", 35.0);
		subject.put("move",subjectMove);
		subject.put("identifier", "GC4x4MB_6");
		subjects.put(subject);
		
		criteria.put("target",target);
		criteria.put("subjects", subjects);
		
		
		//hal.translateProductStep(new ProductStep(1, criteria, new Service("place")));
		JSONObject staticSettings = hal.deleteModule(new ModuleIdentifier("HU", "delta_robot_type_B", "1"));
		System.out.println(staticSettings.toString());
	}
	
	@Override
	public void onTranslationFinished(String service, JSONObject criteria, ArrayList<HardwareStep> hardwareStep) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hardwareSteps.addAll(hardwareStep);// = hardwareStep;
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
