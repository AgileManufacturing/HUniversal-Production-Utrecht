package HAL.testerClasses;

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
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;

public class HALTesterClassStewartGough implements HardwareAbstractionLayerListener {
	HardwareAbstractionLayer hal;
	JSONObject criteria1 = new JSONObject();
	JSONObject criteria2 = new JSONObject();
	boolean state = false;

	static String equipletName = "EQ3";
	static final String baseDir = "generatedOutput/";

	// six axis
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
			"				\"boundaryBoxMinX\" : -200.0," +
			"				\"boundaryBoxMaxX\" : 200.0," +
			"				\"boundaryBoxMinY\" : -200.0," +
			"				\"boundaryBoxMaxY\" : 200.0," +
			"				\"boundaryBoxMinZ\" : -330.0," +
			"				\"boundaryBoxMaxZ\" : -180.0" +
			"			}," +
			"			\"contactSensorToZeroAngleDegrees\" : 20.0," +
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
			"			\"command\":\"rosrun stewart_gough_node stewart_gough_node {isSimulated} {isshadow} {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
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
			"			\"parentLink\":\"base\"," +
			"			\"childLink\":\"effector\"," +
			"			\"childLinkOffsetX\":0.0," +
			"			\"childLinkOffsetY\":0.0," +
			"			\"childLinkOffsetZ\":0.0" +
			"		}," +
			"		\"supportedMutations\": [" +
			"			\"move\", \"rotate\"" +
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
			"\"mountPointY\":2" +
			"}";
	// gripper
	static String moduleB_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"gripper_type_A\"," +
			"		\"serialNumber\":\"2\"," +
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
			"			\"command\":\"rosrun gripper_node gripper_node {isSimulated} {isshadow} {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
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
			"			\"parentLink\":\"base\"," +
			"			\"childLink\":\"base\"," +
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
			"		\"serialNumber\":\"2\"," +
			"	}," +
			"	\"type\":{" +
			"		\"properties\":{}," +
			"		\"rosSoftware\":{" +
			"			\"buildNumber\":1," +
			"			\"rosFile\": \"";
	static String moduleC_02 = "\"," +
			"			\"command\":\"roslaunch camera.launch isSimulated:={isSimulated} isShadow:={isShadow} equipletName:={equipletName} manufacturer:={manufacturer} typeNumber:={typeNumber} serialNumber:={serialNumber}\"" +
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
	static String moduleD_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\"," +
			"		\"typeNumber\":\"Cheap_ass_lens\"," +
			"		\"serialNumber\":\"2\"," +
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
			"		\"serialNumber\":\"2\"" +
			"	}," +
			"	\"mountPointX\":null," +
			"	\"mountPointY\":null" +
			"}";
	// workplane
	static String moduleE_01 = "{" +
			"	\"moduleIdentifier\":{" +
			"		\"manufacturer\":\"HU\"," +
			"		\"typeNumber\":\"workplane_type_A\"," +
			"		\"serialNumber\":\"2\"," +
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
			"			\"command\":\"rosrun part_locator_node part_locator_node {isSimulated} {isshadow} {equipletName} {manufacturer} {typeNumber} {serialNumber}\"" +
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
		if(args.length >= 1) {
			equipletName = args[0];
		}
		System.out.println("Inserting equiplet " + equipletName);
		
		Logger.log(LogSection.HAL, LogLevel.DEBUG, "Starting");
		@SuppressWarnings("unused")
		HALTesterClassStewartGough htc = new HALTesterClassStewartGough();
		htc = null;
	}
	public HALTesterClassStewartGough() throws KnowledgeException, BlackboardUpdateException, IOException, JSONException, InvalidMastModeException {
		hal = new HardwareAbstractionLayer(this);

		FileInputStream fis;
		byte[] content;

		File stewartGoughHal = new File(baseDir + "HAL/modules/" + "StewartGough.jar");
		fis = new FileInputStream(stewartGoughHal);
		content = new byte[(int) stewartGoughHal.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotHal = new String(Base64.encodeBase64(content));
		
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
		
		File gripperHal = new File(baseDir + "HAL/modules/" + "Gripper.jar");
		fis = new FileInputStream(gripperHal);
		content = new byte[(int) gripperHal.length()];
		fis.read(content);
		fis.close();
		String base64GripperHal = new String(Base64.encodeBase64(content));
		
		File stewartGoughRos = new File(baseDir + "nodes/" + "stewart_gough.zip");
		fis = new FileInputStream(stewartGoughRos);
		content = new byte[(int) stewartGoughRos.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotRos = new String(Base64.encodeBase64(content));
		
		File gripperRos = new File(baseDir + "nodes/" + "gripper.zip");
		fis = new FileInputStream(gripperRos);
		content = new byte[(int) gripperRos.length()];
		fis.read(content);
		fis.close();
		String base64GripperRos = new String(Base64.encodeBase64(content));
		
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
		
		File stewartGoughGazebo = new File(baseDir + "models/" + "sixAxis.zip");
		fis = new FileInputStream(stewartGoughGazebo);
		content = new byte[(int) stewartGoughGazebo.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotGazebo = new String(Base64.encodeBase64(content));
		
		File gripperGazebo = new File(baseDir + "models/" + "gripper.zip");
		fis = new FileInputStream(gripperGazebo);
		content = new byte[(int) gripperGazebo.length()];
		fis.read(content);
		fis.close();
		String base64GripperGazebo = new String(Base64.encodeBase64(content));
		
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
		
		
		File drawHal = new File(baseDir + "HAL/capabilities/" + "Draw.jar");
		fis = new FileInputStream(drawHal);
		content = new byte[(int) drawHal.length()];
		fis.read(content);
		fis.close();
		String base64Draw = new String(Base64.encodeBase64(content));
		
		File pickAndPlaceWithRotationHal = new File(baseDir + "HAL/capabilities/" + "PickAndPlaceWithRotation.jar");
		fis = new FileInputStream(pickAndPlaceWithRotationHal);
		content = new byte[(int) pickAndPlaceWithRotationHal.length()];
		fis.read(content);
		fis.close();
		String base64PickAndPlaceWithRotation = new String(Base64.encodeBase64(content));
		
		
		// six axsi
		String moduleA = moduleA_01 + base64DeltaRobotRos + moduleA_02 + base64DeltaRobotHal + 
				moduleA_03 + base64DeltaRobotGazebo + moduleA_04 + base64Draw + moduleA_05 + base64PickAndPlaceWithRotation + moduleA_06; 
		JSONObject a = new JSONObject(new JSONTokener(moduleA));
		
		// gripper
		String moduleB = moduleB_01 + base64GripperRos + moduleB_02 + base64GripperHal + 
				moduleB_03 + base64GripperGazebo + moduleB_04; 
		JSONObject b = new JSONObject(new JSONTokener(moduleB));
		
		// camera
		String moduleC = moduleC_01 + base64CameraRos + moduleC_02 + base64PenHal + 
				moduleC_03 + base64CameraGazebo + moduleC_04;
		JSONObject c = new JSONObject(new JSONTokener(moduleC));
		
		// lens
		// TODO fix non hal software
		String moduleD = moduleD_01 + "" + moduleD_02 + base64LensGazebo + moduleD_03;
		JSONObject d = new JSONObject(new JSONTokener(moduleD));
		
		// workplane
		String moduleE = moduleE_01 + base64WorkplaneRos + moduleE_02 + base64WorkplaneHal + 
				moduleE_03 + base64WorkplaneGazebo + moduleE_04;
		JSONObject e = new JSONObject(new JSONTokener(moduleE));
		
		
		hal.insertModule(a, a);
		hal.insertModule(b, b);
		hal.insertModule(c, c);
		hal.insertModule(d, d);
		hal.insertModule(e, e);
		
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


		hal.shutdown();
		hal = null;
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
