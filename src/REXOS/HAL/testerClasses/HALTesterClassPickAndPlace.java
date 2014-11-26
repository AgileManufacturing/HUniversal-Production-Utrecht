package HAL.testerClasses;

import generic.ProductStep;
import generic.Service;
import jade.core.AID;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.codec.binary.Base64;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.BlackboardHandler;
import HAL.HardwareAbstractionLayer;
import HAL.Module;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.HardwareStep.HardwareStepStatus;
import MAS.equiplet.Job;
import MAS.util.Tick;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

public class HALTesterClassPickAndPlace implements HardwareAbstractionLayerListener {
	static HALTesterClassPickAndPlace htc = new HALTesterClassPickAndPlace();
	static HardwareAbstractionLayer hal;
	static BlackboardHandler blackboardUpdated;
	static JSONObject criteria1 = new JSONObject();
	static JSONObject criteria2 = new JSONObject();
	boolean state = false;
	
	static final String baseDir = "C:/git/HUniversal-Production-Utrecht/";
	
	// delta robot
	static String moduleA_01 = "{"
			+ "	\"manufacturer\":\"HU\","
			+ "	\"typeNumber\":\"delta_robot_type_B\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"{"
			+ "	\\\"midPointX\\\" : 75.0,"
			+ "	\\\"midPointY\\\" : -200.0,"
			+ "	\\\"midPointZ\\\" : -29.5,"
			+ "	\\\"deltaRobotMeasures\\\" : {"
			+ "		\\\"baseRadius\\\" : 101.3,"
			+ "		\\\"hipLength\\\" : 100.0,"
			+ "		\\\"effectorRadius\\\" : 46.19,"
			+ "		\\\"ankleLength\\\" : 300.0,"
			+ "		\\\"hipAnleMaxAngleDegrees\\\" : 22.0,"
			+ "		\\\"motorFromZeroToTopAngleDegrees\\\" : 20.0,"
			+ "		\\\"boundaryBoxMinX\\\" : -200.0,"
			+ "		\\\"boundaryBoxMaxX\\\" : 200.0,"
			+ "		\\\"boundaryBoxMinY\\\" : -200.0,"
			+ "		\\\"boundaryBoxMaxY\\\" : 200.0,"
			+ "		\\\"boundaryBoxMinZ\\\" : -450.0,"
			+ "		\\\"boundaryBoxMaxZ\\\" : -180.0"
			+ "	},"
			+ "	\\\"calibrationBigStepFactor\\\" : 20,"
			+ "	\\\"stepperMotorProperties\\\" : {"
			+ "		\\\"motorMinAngleDegrees\\\" : -18.0,"
			+ "		\\\"motorMaxAngleDegrees\\\" : 90.0,"
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
			+ "			\"command\":\"rosrun delta_robot_node delta_robot_node {equipletName} {manufacturer} {typeNumber} {serialNumber}\""
			+ "		},"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleA_03 = "\","
			+ "			\"className\":\"HAL.modules.DeltaRobot\""
			+ "		},"
			+ "		\"supportedMutations\": ["
			+ "			\"move\""
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
			+ "				\"name\":\"PickAndPlace\","
			+ "				\"treeNumber\":1,"
			+ "				\"halSoftware\":{"
			+ "					\"buildNumber\":1,"
			+ "					\"jarFile\": \"";
	static String moduleA_05 = "\","
			+ "					\"className\":\"HAL.capabilities.PickAndPlace\""
			+ "				},"
			+ "				\"requiredMutationsTrees\":["
			+ "					{"
			+ "						\"treeNumber\":1,"
			+ "						\"mutations\":["
			+ "							\"move\", \"pick\", \"place\""
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
			+ "	\\\"modbusIp\\\" : \\\"192.168.0.22\\\","
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
			+ "\"mountPointY\":1"
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
			+ " \\\"gripperSize\\\" : 31.00,"
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
			+ "	\\\"modbusIp\\\" : \\\"192.168.0.22\\\","
			+ "	\\\"modbusPort\\\" : 502"
			+ "}\","
			+ "	\"calibrationData\":["
			+ "	],"
			+ "	\"attachedTo\":{"
			+ "		\"manufacturer\":\"HU\","
			+ "		\"typeNumber\":\"delta_robot_type_B\","
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
			+ "     \\\"midPointZ\\\" : 34.8,"
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
			+ "\"mountPointY\":10"
			+ "}";

			
	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		
		Logger.log(LogSection.NONE, LogLevel.DEBUG, "Starting");
		
		hal = new HardwareAbstractionLayer(htc);
		
		FileInputStream fis;
		byte[] content;

		File deltaRobotJar = new File(baseDir + "DeltaRobot.jar");
		fis = new FileInputStream(deltaRobotJar);
		content = new byte[(int) deltaRobotJar.length()];
		fis.read(content);
		fis.close();
		String base64Module = new String(Base64.encodeBase64(content));
		
		File deltaRobotZip = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(deltaRobotZip);
		content = new byte[(int) deltaRobotZip.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotRos = new String(Base64.encodeBase64(content));
		
		File gripperZip = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(gripperZip);
		content = new byte[(int) gripperZip.length()];
		fis.read(content);
		fis.close();
		String base64GripperRos = new String(Base64.encodeBase64(content));
		
		File cameraZip = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(cameraZip);
		content = new byte[(int) cameraZip.length()];
		fis.read(content);
		fis.close();
		String base64CameraRos = new String(Base64.encodeBase64(content));
		
		File workplaneZip = new File(baseDir + "nodes.zip");
		fis = new FileInputStream(workplaneZip);
		content = new byte[(int) workplaneZip.length()];
		fis.read(content);
		fis.close();
		String base64WorkplaneRos = new String(Base64.encodeBase64(content));
		
		File penJar = new File(baseDir + "Pen.jar");
		fis = new FileInputStream(penJar);
		content = new byte[(int) penJar.length()];
		fis.read(content);
		fis.close();
		String base64Pen = new String(Base64.encodeBase64(content));
		
		File gripperJar = new File(baseDir + "Gripper.jar");
		fis = new FileInputStream(gripperJar);
		content = new byte[(int) gripperJar.length()];
		fis.read(content);
		fis.close();
		String base64Gripper = new String(Base64.encodeBase64(content));
		
		File drawJar = new File(baseDir + "Draw.jar");
		fis = new FileInputStream(drawJar);
		content = new byte[(int) drawJar.length()];
		fis.read(content);
		fis.close();
		String base64Draw = new String(Base64.encodeBase64(content));
		
		File pickAndPlaceJar = new File(baseDir + "PickAndPlace.jar");
		fis = new FileInputStream(pickAndPlaceJar);
		content = new byte[(int) pickAndPlaceJar.length()];
		fis.read(content);
		fis.close();
		String base64PickAndPlace = new String(Base64.encodeBase64(content));
		
		// deltarobot
		String moduleA = moduleA_01 + base64DeltaRobotRos + moduleA_02 + base64Module + moduleA_03 + base64Draw + moduleA_04 + base64PickAndPlace + moduleA_05; 
		JSONObject a = new JSONObject(new JSONTokener(moduleA));
		hal.insertModule(a, a);
		
//		// pen
//		String moduleB = moduleB_01 + base64Pen + moduleB_02; 
//		JSONObject b = new JsonParser().parse(moduleB).getAsJSONObject();
//		hal.insertModule(b, b);
		
		// gripper
		String moduleB = moduleB_01 + base64GripperRos + moduleB_02 + base64Gripper + moduleB_03; 
		JSONObject b = new JSONObject(new JSONTokener(moduleB));
		hal.insertModule(b, b);
		
		// camera
		String moduleC = moduleC_01 + base64CameraRos + moduleC_02 + base64Pen + moduleC_03;
		JSONObject c = new JSONObject(new JSONTokener(moduleC));
		hal.insertModule(c, c);
		
		// lens
		String moduleD = moduleD_01 + base64Pen + moduleD_02; 
		JSONObject d = new JSONObject(new JSONTokener(moduleD));
		hal.insertModule(d, d);
		
		// workplane
		String moduleE = moduleE_01 + base64WorkplaneRos + moduleE_02 + base64Pen + moduleE_03;
		JSONObject e = new JSONObject(new JSONTokener(moduleE));
		hal.insertModule(e, e);
		
		
		//Bakje 3 GOED MORE TEST
		//double falsex = -2.2;
		//double falsey = 2.4;
		
		JSONObject target1 = new JSONObject();
		JSONObject targetMove1 = new JSONObject();
		targetMove1.put("x", 0);
		targetMove1.put("y", 0);
		targetMove1.put("z", 0);
		JSONObject targetMove1Approach = new JSONObject(); 
		targetMove1Approach.put("x", 0);
		targetMove1Approach.put("y", 0);
		targetMove1Approach.put("z", 15);
		targetMove1.put("approach", targetMove1Approach);
		target1.put("move",targetMove1);
		target1.put("identifier", "GC4x4MB_1");
		
		JSONArray subjects1 = new JSONArray();
		JSONObject subject1 = new JSONObject();
		JSONObject subjectMove1 = new JSONObject();
		subjectMove1.put("x", 0);
		subjectMove1.put("y", 0);
		subjectMove1.put("z", 0);
		JSONObject subjectMove1Approach = new JSONObject(); 
		subjectMove1Approach.put("x", 0);
		subjectMove1Approach.put("y", 0);
		subjectMove1Approach.put("z", 15);
		subjectMove1.put("approach", subjectMove1Approach);
		subject1.put("move",subjectMove1);
		subject1.put("identifier", "GC4x4MB_6");
		subjects1.put(subject1);
		
		JSONObject target2 = new JSONObject();
		JSONObject targetMove2 = new JSONObject();
		targetMove2.put("x", 0);
		targetMove2.put("y", 0);
		targetMove2.put("z", 0);
		JSONObject targetMove2Approach = new JSONObject(); 
		targetMove2Approach.put("x", 0);
		targetMove2Approach.put("y", 0);
		targetMove2Approach.put("z", 15);
		targetMove2.put("approach", targetMove2Approach);
		target2.put("move",targetMove2);
		target2.put("identifier", "GC4x4MB_6");
		
		JSONArray subjects2 = new JSONArray();
		JSONObject subject2 = new JSONObject();
		JSONObject subjectMove2 = new JSONObject();
		subjectMove2.put("x", 0);
		subjectMove2.put("y", 0);
		subjectMove2.put("z", 0);
		JSONObject subjectMove2Approach = new JSONObject(); 
		subjectMove2Approach.put("x", 0);
		subjectMove2Approach.put("y", 0);
		subjectMove2Approach.put("z", 15);
		subjectMove2.put("approach", subjectMove2Approach);
		subject2.put("move",subjectMove2);
		subject2.put("identifier", "GC4x4MB_1");
		subjects2.put(subject2);
		
		criteria1.put("target",target1);
		criteria1.put("subjects", subjects1);	
		
		criteria2.put("target",target2);
		criteria2.put("subjects", subjects2);	
		System.out.println("Done Adding Modules");
	/*	Map<String, Object> crit = new HashMap<String, Object>();
		crit.put("target",target1);
		crit.put("subjects", subjects1);*/
		//hal.translateProductStep(
		//		new Job(1, "place", "", crit , new Tick(1), new Tick(1), new Tick(1)));

	}
	
	/*@Override
	public void onTranslationFinished(Job job, ArrayList<HardwareStep> hardwareSteps) {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Translation finished");
		hal.executeHardwareSteps(hardwareSteps);
	}*/

	//@Override
	//public void onTranslationFailed(/*Job job*/) {
		//Logger.log(LogSection.NONE, LogLevel.NOTIFICATION, "Translation failed of the following product step:", job);
	//}
	
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

	@SuppressWarnings("deprecation")
	@Override
	public void onExecutionFinished() {
		Logger.log(LogSection.NONE, LogLevel.INFORMATION, "Execution finished");
		if(state){
			state = false;
			//hal.translateProductStep(
		//			new Job(1, "place", "Product-Agent-0", null , new Tick(1), new Tick(1), new Tick(1)));
		}else{
			state = true;
		//	hal.translateProductStep(
		//			new Job(1, "place", "Product-Agent-0", null , new Tick(1), new Tick(1), new Tick(1)));
		}
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

	@Override
	public void onTranslationFinished(String service,
			ArrayList<HardwareStep> hardwareSteps) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onTranslationFailed(String service, JSONObject criteria) {
		// TODO Auto-generated method stub
		
	}

}
