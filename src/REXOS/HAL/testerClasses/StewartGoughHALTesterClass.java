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
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

public class StewartGoughHALTesterClass implements HardwareAbstractionLayerListener {
	static StewartGoughHALTesterClass htc = new StewartGoughHALTesterClass();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	static BlackboardHandler blackboardUpdated;
	static JSONObject criteria;
	static boolean insert = true;
	
	static final String baseDir = "jars/";
	static final String Aridir = "C:/users/Aristides/Desktop/Six Axis/";
	static final String dir = baseDir;
	
	// delta robot
	static String moduleA_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"six_axis_type_A\"," +
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
			"				\"ankleLength\" : 250.0," +
			"				\"hipAnleMaxAngleDegrees\" : 22.0," +
			"				\"motorFromZeroToTopAngleDegrees\" : 20.0," +
			"				\"boundaryBoxMinX\" : -200.0," +
			"				\"boundaryBoxMaxX\" : 200.0," +
			"				\"boundaryBoxMinY\" : -200.0," +
			"				\"boundaryBoxMaxY\" : 200.0," +
			"				\"boundaryBoxMinZ\" : -330.0," +
			"				\"boundaryBoxMaxZ\" : -180.0" +
			"			}," +
			"			\"calibrationBigStepFactor\" : 20," +
			"			\"stepperMotorProperties\" : {" +
			"				\"motorMinAngleDegrees\" : -18.0," +
			"				\"motorMaxAngleDegrees\" : 80.0," +
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
			"			\"command\":\"rosrun stewart_gough_node stewart_gough_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
			"		}," +
			"		\"halSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"jarFile\": \"";
	static String moduleA_03 = "\"," +
			"			\"className\":\"HAL.modules.StewartGough\"" +
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
			"				\"name\":\"PickAndPlaceWithRotation\"," +
			"				\"treeNumber\":1," +
			"				\"halSoftware\":{" +
			"					\"buildNumber\":1," +
			"					\"jarFile\": \"";
	static String moduleA_06 = "\"," +
			"					\"className\":\"HAL.capabilities.PickAndPlaceWithRotation\"" +
			"				}," +
			"				\"requiredMutationsTrees\":[" +
			"					{" +
			"						\"treeNumber\":1," +
			"						\"mutations\":[" +
			"							\"move\", \"rotate\", \"pick\", \"place\"" +
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
			"		\"modbusIp\" : \"192.168.0.32\"," +
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
			"\"mountPointY\":0" +
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
			"			\"childLink\":\"otherLink\"" +
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
			"		\"typeNumber\":\"six_axis_type_A\"," +
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
			"			\"childLink\":\"otherLink\"" +
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
			"			\"childLink\":\"otherLink\"" +
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
			"			\"childLink\":\"otherLink\"" +
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
	
	
	public static void main(String[] args) throws Exception {
		System.out.println("Starting");
		
		
		// TODO Auto-generated method stub
		hal = new HardwareAbstractionLayer(htc);

	if(insert){		
		FileInputStream fis;
		byte[] content;

		File sixAxistHal = new File(baseDir + "StewartGough.jar");
		fis = new FileInputStream(sixAxistHal);
		content = new byte[(int) sixAxistHal.length()];
		fis.read(content);
		fis.close();
		String base64SixAxistHal = new String(Base64.encodeBase64(content));
		
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
		
		File pickAndPlaceWithRotationHal = new File(baseDir + "PickAndPlaceWithRotation.jar");
		fis = new FileInputStream(pickAndPlaceWithRotationHal);
		content = new byte[(int) pickAndPlaceWithRotationHal.length()];
		fis.read(content);
		fis.close();
		String base64PickAndPlaceWithRotation = new String(Base64.encodeBase64(content));
		
		
		// deltarobot
		String moduleA = moduleA_01 + base64DeltaRobotRos + moduleA_02 + base64SixAxistHal + 
				moduleA_03 + base64DeltaRobotGazebo + moduleA_04 + base64Draw + moduleA_05 + base64PickAndPlaceWithRotation + moduleA_06; 
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
		
		insert = false;
	}
		
/*criteria = new JSONObject();
		JSONObject target = new JSONObject();
		JSONObject targetMove = new JSONObject();
		JSONObject targetRotate = new JSONObject();
		targetMove.addProperty("x", 5.0);
		targetMove.addProperty("y", 5.0);
		targetMove.addProperty("z", 20.0);
		targetMove.addProperty("maxAcceleration", 5);
		
		targetRotate.addProperty("x", 0);
		targetRotate.addProperty("y", 0);
		targetRotate.addProperty("z", 0);
		target.add("move",targetMove);
		target.add("rotate",targetRotate);
		target.addProperty("identifier", "GC4x4MB_1");
		
		JSONArray subjects = new JSONArray();
		JSONObject subject = new JSONObject();
		JSONObject subject1 = new JSONObject();
		JSONObject subjectMove1 = new JSONObject();
		JSONObject subjectRotate1 = new JSONObject();
		JSONObject subjectMove = new JSONObject();
		JSONObject subjectRotate = new JSONObject();
		subjectMove.addProperty("x", 3.5);
		subjectMove.addProperty("y", 3.5);
		subjectMove.addProperty("z", 20.0);
		subjectMove.addProperty("maxAcceleration", 5);
		
		subjectRotate.addProperty("x", 0);
		subjectRotate.addProperty("y", 0);
		subjectRotate.addProperty("z", 0);
		
		
		subject.add("move",subjectMove);
		subject.add("rotate",subjectRotate);
		subject.addProperty("identifier", "GC4x4MB_6");
		subjects.add(subject);
		
		//Test Alex
		subjectMove1.addProperty("x", 5.0);
		subjectMove1.addProperty("y", 5.0);
		subjectMove1.addProperty("z", 20.0);
		subjectMove1.addProperty("maxAcceleration", 5);
		
		subjectRotate1.addProperty("x", 0);
		subjectRotate1.addProperty("y", 0);
		subjectRotate1.addProperty("z", 0);
		
		
		subject1.add("move",subjectMove1);
		subject1.add("rotate",subjectRotate1);
		subject1.addProperty("identifier", "GC4x4MB_3");
		subjects.add(subject1);
		
		criteria.add("target",target);
		criteria.add("subjects",subjects);
		
		
		//hal.translateProductStep(new ProductStep("1", criteria, new Service("place")));
		
		Service service = new Service("PickAndPlace");
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
		Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, "Translation failed of the following product step:", new Object[] {
				service, criteria });
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
		hal.translateProductStep("place", criteria);
	}

	@Override
	public void onEquipletMachineStateChanged(String state) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The state of equiplet " + getEquipletName()
				+ " has changed to " + state);
	}

	@Override
	public void onEquipletModeChanged(String mode) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "The mode of equiplet " + getEquipletName()
				+ " has changed to " + mode);
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
