package HAL.testerClasses;

import generic.ProductStep;
import generic.Service;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.BlackboardHandler;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;

import org.json.JSONObject;
import org.json.JSONTokener;

public class StewartGoughHALTesterClass implements HardwareAbstractionLayerListener {
	static StewartGoughHALTesterClass htc = new StewartGoughHALTesterClass();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	static BlackboardHandler blackboardUpdated;
	static JSONObject criteria;
	static boolean insert = true;
	
	static final String baseDir = "/home/agileman/Desktop/";
	static final String Aridir = "C:/users/Aristides/Desktop/Six Axis/";
	static final String dir = baseDir;
	
	// six_axis robot
	static String moduleA_01 = "{"
			+ "	\"manufacturer\":\"HU\","
			+ "	\"typeNumber\":\"six_axis_type_A\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"{"
			+ "	\\\"midPointX\\\" : 75.0,"
			+ "	\\\"midPointY\\\" : -200.0,"
			+ "	\\\"midPointZ\\\" : -35.3,"
			+ "	\\\"stewartGoughMeasures\\\" : {"
			+ "		\\\"baseRadius\\\" : 101.3,"
			+ "		\\\"hipLength\\\" : 100.0,"
			+ "		\\\"effectorRadius\\\" : 46.19,"
			+ "		\\\"ankleLength\\\" : 250.0,"
			+ "		\\\"hipAnleMaxAngleDegrees\\\" : 22.0,"
			+ "		\\\"motorFromZeroToTopAngleDegrees\\\" : 20.0,"
			+ "		\\\"boundaryBoxMinX\\\" : -200.0,"
			+ "		\\\"boundaryBoxMaxX\\\" : 200.0,"
			+ "		\\\"boundaryBoxMinY\\\" : -200.0,"
			+ "		\\\"boundaryBoxMaxY\\\" : 200.0,"
			+ "		\\\"boundaryBoxMinZ\\\" : -330.0,"
			+ "		\\\"boundaryBoxMaxZ\\\" : -180.0"
			+ "	},"
			+ "	\\\"calibrationBigStepFactor\\\" : 20,"
			+ "	\\\"stepperMotorProperties\\\" : {"
			+ "		\\\"motorMinAngleDegrees\\\" : -18.0,"
			+ "		\\\"motorMaxAngleDegrees\\\" : 80.0,"
			+ "		\\\"microStepAngleDegrees\\\" : 0.036,"
			+ "		\\\"minAccelerationDegrees\\\" : 36,"
			+ "		\\\"maxAccelerationDegrees\\\" : 36000,"
			+ "		\\\"minSpeedDegrees\\\" : 0.036,"
			+ "		\\\"maxSpeedDegrees\\\" : 18000"
			+ "	}"
			+ "}\","
			+ "		\"rosSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"rosFile\": \"";
	static String moduleA_02 = "\","
			+ "			\"command\":\"rosrun stewart_gough_node stewart_gough_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\""
			+ "		},"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleA_03 = "\","
			+ "			\"className\":\"HAL.modules.StewartGough\""
			+ "		},"
			+ "		\"supportedMutations\": ["
			+ "			\"move\",\"rotate\""
			+ "		],"
			+ "		\"capabilities\":["
			+ "			{"
			+ "				\"name\":\"Draw\","
			+ "				\"treeNumber\":1,"
			+ "				\"halSoftware\":{"
			+ "					\"buildNumber\":1,"
			+ "					\"jarFile\": \"";
	static String moduleA_04 = "\","
			+ "					\"className\":\"HAL.capabilities.Draw\""
			+ "				},"
			+ "				\"requiredMutationsTrees\":["
			+ "					{"
			+ "						\"treeNumber\":1,"
			+ "						\"mutations\":["
			+ "							\"move\", \"draw\""
			+ "						]"
			+ "					}"
			+ "				],"
			+ "				\"services\":["
			+ "					\"draw\""
			+ "				]"
			+ "			},"
			+ "			{"
			+ "				\"name\":\"PickAndPlaceWithRotation\","
			+ "				\"treeNumber\":1,"
			+ "				\"halSoftware\":{"
			+ "					\"buildNumber\":1,"
			+ "					\"jarFile\": \"";
	static String moduleA_05 = "\","
			+ "					\"className\":\"HAL.capabilities.PickAndPlaceWithRotation\""
			+ "				},"
			+ "				\"requiredMutationsTrees\":["
			+ "					{"
			+ "						\"treeNumber\":1,"
			+ "						\"mutations\":["
			+ "							\"move\",\"rotate\",\"pick\", \"place\""
			+ "						]"
			+ "					}"
			+ "				],"
			+ "				\"services\":["
			+ "					\"place\""
			+ "				]"
			+ "			}"
			+ "		]"
			+ "	},"
			+ "	\"properties\":\"{"
			+ "	\\\"modbusIp\\\" : \\\"192.168.0.32\\\","
			+ "	\\\"modbusPort\\\" : 502"
			+ "}\","
			+ "	\"calibrationData\":["
			+ "		{"
			+ "			\"date\":\"2014-01-01\","
			+ "			\"data\":\"aapkip\","
			+ "			\"moduleSet\":["
			+ "				{"
			+ "					\"manufacturer\":\"manA\","
			+ "					\"typeNumber\":\"typeA\","
			+ "					\"serialNumber\":\"serA\""
			+ "				}"
			+ "			]"
			+ "		}"
			+ "	],"
			+ "	\"attachedTo\":null,"
			+ "\"mountPointX\":3,"
			+ "\"mountPointY\":0"
			+ "}";
	// pen
	static String moduleB_01 = "{"
			+ "	\"manufacturer\":\"HU\","
			+ "	\"typeNumber\":\"gripper_type_A\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"{"
			+ "	\\\"modbusAddress\\\" : 8001,"
			+ "	\\\"modbusDevicePin\\\" : 0,"
			+ "	\\\"gripperEnabledMaxSeconds\\\" : 60,"
			+ "	\\\"gripperEnabledWarningSeconds\\\" : 50,"
			+ "	\\\"gripperEnabledCooldownSeconds\\\" : 180,"
			+ "	\\\"watchdogInterval\\\" : 100"
			+ "}\","
			+ "		\"rosSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"rosFile\": \"";
	static String moduleB_02 = "\","
			+ "			\"command\":\"rosrun gripper_node gripper_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\""
			+ "		},"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleB_03 = "\","
			+ "			\"className\":\"HAL.modules.Gripper\""
			+ "		},"
			+ "		\"supportedMutations\": ["
			+ "			\"pick\", \"place\""
			+ "		],"
			+ "		\"capabilities\":["
			+ "		]"
			+ "	},"
			+ "	\"properties\":\"{"
			+ "	\\\"modbusIp\\\" : \\\"192.168.0.32\\\","
			+ "	\\\"modbusPort\\\" : 502"
			+ "}\","
			+ "	\"calibrationData\":["
			+ "	],"
			+ "	\"attachedTo\":{"
			+ "		\"manufacturer\":\"HU\","
			+ "		\"typeNumber\":\"six_axis_type_A\","
			+ "		\"serialNumber\":\"1\""
			+ "	},"
			+ "\"mountPointX\":null,"
			+ "\"mountPointY\":null"
			+ "}";
	// camera
	static String moduleC_01 = "{"
			+ "	\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\","
			+ "	\"typeNumber\":\"DFK_22AUC03\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"\","
			+ "		\"rosSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"rosFile\": \"";
	static String moduleC_02 = "\","
			+ "			\"command\":\"roslaunch camera.launch equipletName:={equipletName} manufacturer:={manufacturer} typeNumber:={typeNumber} serialNumber:={serialNumber}\""
			+ "		},"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleC_03 = "\","
			+ "			\"className\":\"HAL.modules.Camera\""
			+ "		},"
			+ "		\"supportedMutations\": ["
			+ "		],"
			+ "		\"capabilities\":["
			+ "		]"
			+ "	},"
			+ "	\"properties\":\"\","
			+ "	\"calibrationData\":["
			+ "	],"
			+ "	\"attachedTo\":null,"
			+ "\"mountPointX\":3,"
			+ "\"mountPointY\":16"
			+ "}";
	// lens
	static String moduleD_01 = "{"
			+ "	\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\","
			+ "	\"typeNumber\":\"Cheap_ass_lens\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"\","
			+ "		\"rosSoftware\":null,"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleD_02 = "\","
			+ "			\"className\":\"HAL.modules.Lens\""
			+ "		},"
			+ "		\"supportedMutations\": ["
			+ "		],"
			+ "		\"capabilities\":["
			+ "		]"
			+ "	},"
			+ "	\"properties\":\"\","
			+ "	\"calibrationData\":["
			+ "	],"
			+ "	\"attachedTo\":{"
			+ "		\"manufacturer\":\"The_Imaging_Source_Europe_GmbH\","
			+ "		\"typeNumber\":\"DFK_22AUC03\","
			+ "		\"serialNumber\":\"1\""
			+ "	},"
			+ "\"mountPointX\":null,"
			+ "\"mountPointY\":null"
			+ "}";
	// workplane
	static String moduleE_01 = "{"
			+ "     \"manufacturer\":\"HU\","
			+ "     \"typeNumber\":\"workplane_type_A\","
			+ "     \"serialNumber\":\"1\","
			+ "     \"type\":{"
			+ "	     \"properties\":\"{"
			+ "     \\\"midPointX\\\" : 175.0,"
			+ "     \\\"midPointY\\\" : -200.0,"
			+ "     \\\"midPointZ\\\" : 33.33,"
			+ "     \\\"topLeftValue\\\" : \\\"_WP_TL\\\","
			+ "     \\\"topRightValue\\\" : \\\"_WP_TR\\\","
			+ "     \\\"bottomRightValue\\\" : \\\"_WP_BR\\\","
			+ "     \\\"workPlaneWidth\\\" : 80.0,"
			+ "     \\\"workPlaneHeight\\\" : 80.0"
			+ "}\","
			+ "	     \"rosSoftware\":{"
			+ "		     \"buildNumber\":1,"
			+ "		     \"rosFile\": \"";
	static String moduleE_02 = "\","
			+ "		     \"command\":\"rosrun part_locator_node part_locator_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\""
			+ "	     },"
			+ "	     \"halSoftware\":{"
			+ "		     \"buildNumber\":1,"
			+ "		     \"jarFile\": \"";
	static String moduleE_03 = "\","
			+ "		     \"className\":\"HAL.modules.Workplane\""
			+ "	     },"
			+ "	     \"supportedMutations\": ["
			+ "	     ],"
			+ "	     \"capabilities\":["
			+ "	     ]"
			+ "     },"
			+ "     \"properties\":\"\","
			+ "     \"calibrationData\":["
			+ "     ],"
			+ "     \"attachedTo\":null,"
			+ "\"mountPointX\":1,"
			+ "\"mountPointY\":9"
			+ "}";
	
	
	public static void main(String[] args) throws Exception {
		System.out.println("Starting");
		
		
		// TODO Auto-generated method stub
		hal = new HardwareAbstractionLayer(htc);

	if(insert){		
	
		FileInputStream fis;
		byte[] content;
		System.out.println("Start insertig module");
		
		File stewartGoughJar = new File(dir + "StewartGough.jar");
		fis = new FileInputStream(stewartGoughJar);
		content = new byte[(int) stewartGoughJar.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobot = new String(Base64.encodeBase64(content));
		
		File stewartGoughZip = new File(dir + "nodes.zip");
		fis = new FileInputStream(stewartGoughZip);
		content = new byte[(int) stewartGoughZip.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotRos = new String(Base64.encodeBase64(content));
		
		File gripperZip = new File(dir + "nodes.zip");
		fis = new FileInputStream(gripperZip);
		content = new byte[(int) gripperZip.length()];
		fis.read(content);
		fis.close();
		String base64GripperRos = new String(Base64.encodeBase64(content));
		
		File cameraZip = new File(dir + "nodes.zip");
		fis = new FileInputStream(cameraZip);
		content = new byte[(int) cameraZip.length()];
		fis.read(content);
		fis.close();
		String base64CameraRos = new String(Base64.encodeBase64(content));
		
		File workplaneZip = new File(dir + "nodes.zip");
		fis = new FileInputStream(workplaneZip);
		content = new byte[(int) workplaneZip.length()];
		fis.read(content);
		fis.close();
		String base64WorkplaneRos = new String(Base64.encodeBase64(content));
		
		File penJar = new File(dir + "Pen.jar");
		fis = new FileInputStream(penJar);
		content = new byte[(int) penJar.length()];
		fis.read(content);
		fis.close();
		String base64Pen = new String(Base64.encodeBase64(content));
		
		File gripperJar = new File(dir + "Gripper.jar");
		fis = new FileInputStream(gripperJar);
		content = new byte[(int) gripperJar.length()];
		fis.read(content);
		fis.close();
		String base64Gripper = new String(Base64.encodeBase64(content));
		
		File drawJar = new File(dir + "Draw.jar");
		fis = new FileInputStream(drawJar);
		content = new byte[(int) drawJar.length()];
		fis.read(content);
		fis.close();
		String base64Draw = new String(Base64.encodeBase64(content));
		
		File pickAndPlaceJar = new File(dir + "PickAndPlaceWithRotation.jar");
		fis = new FileInputStream(pickAndPlaceJar);
		content = new byte[(int) pickAndPlaceJar.length()];
		fis.read(content);
		fis.close();
		String base64PickAndPlaceWithRotation = new String(Base64.encodeBase64(content));
		
		System.out.println("Finished reading moduleFiles, starting insertion of modules...");
		
		
		// Six axis
		String moduleA = moduleA_01 + base64DeltaRobotRos + moduleA_02 + base64DeltaRobot + moduleA_03 + base64Draw + moduleA_04 + base64PickAndPlaceWithRotation + moduleA_05; 
		JSONObject a = new JSONObject(new JSONTokener(moduleA));
		hal.insertModule(a, a);
		System.out.println("Module A inserted");
		
	//	// pen
	//	String moduleB = moduleB_01 + base64Pen + moduleB_02; 
	//	JSONObject b = new JsonParser().parse(moduleB).getAsJSONObject();
	//	hal.insertModule(b, b);
		
		// gripper
		String moduleB = moduleB_01 + base64GripperRos + moduleB_02 + base64Gripper + moduleB_03; 
		JSONObject b = new JSONObject(new JSONTokener(moduleB));
		hal.insertModule(b, b);
		System.out.println("Module B inserted");
	
		// camera
		String moduleC = moduleC_01 + base64CameraRos + moduleC_02 + base64Pen + moduleC_03;
		JSONObject c = new JSONObject(new JSONTokener(moduleC));
		hal.insertModule(c, c);
		System.out.println("Module C inserted");
		// lens
		String moduleD = moduleD_01 + base64Pen + moduleD_02; 
		JSONObject d = new JSONObject(new JSONTokener(moduleD));
		hal.insertModule(d, d);
		System.out.println("Module D inserted");
		// workplane
		String moduleE = moduleE_01 + base64WorkplaneRos + moduleE_02 + base64Pen + moduleE_03;
		JSONObject e = new JSONObject(new JSONTokener(moduleE));
		hal.insertModule(e, e);
		System.out.println("Module E inserted");
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
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareStep) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hardwareSteps.addAll(hardwareStep);// = hardwareStep;
		hal.executeHardwareSteps(hardwareSteps);
	}


	@Override
	public void onTranslationFailed(ProductStep productStep) {
		Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, "Translation failed of the following product step:", productStep);
	}

	@Override
	public void onProcessStatusChanged(String status, 
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
		return "EQ3";
	}

	@Override
	public void onExecutionFinished() {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Execution finished");
		hal.translateProductStep(new ProductStep(1 , criteria, new Service("place")));
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
	
}
