package HAL;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;

import HAL.listeners.BlackboardListener;
import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public class StewartGoughHALTesterClass implements HardwareAbstractionLayerListener {
	static StewartGoughHALTesterClass htc = new StewartGoughHALTesterClass();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	static BlackboardHandler blackboardUpdated;
	// delta robot
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
				+ "				\"name\":\"Place\","
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
				+ "\"mountPointY\":2"
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
				+ "	\"properties\":\"name\","
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
				+ "\"mountPointY\":10"
				+ "}";
			
	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		System.out.println("Starting");
		
		// TODO Auto-generated method stub
		hal = new HardwareAbstractionLayer(htc);

		
		FileInputStream fis;
		byte[] content;

		
		System.out.println("Start insertig module");
		
		File stewartGoughJar = new File("C:/Users/Aristides/Desktop/Six Axis/StewartGough.jar");
		fis = new FileInputStream(stewartGoughJar);
		content = new byte[(int) stewartGoughJar.length()];
		fis.read(content);
		fis.close();
		String base64Module = new String(Base64.encodeBase64(content));
		
		File stewartGoughZip = new File("C:/Users/Aristides/Desktop/Six Axis/nodes.zip");
		fis = new FileInputStream(stewartGoughZip);
		content = new byte[(int) stewartGoughZip.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotRos = new String(Base64.encodeBase64(content));
		
		File gripperZip = new File("C:/Users/Aristides/Desktop/Six Axis/nodes.zip");
		fis = new FileInputStream(gripperZip);
		content = new byte[(int) gripperZip.length()];
		fis.read(content);
		fis.close();
		String base64GripperRos = new String(Base64.encodeBase64(content));
		
		File cameraZip = new File("C:/Users/Aristides/Desktop/Six Axis/nodes.zip");
		fis = new FileInputStream(cameraZip);
		content = new byte[(int) cameraZip.length()];
		fis.read(content);
		fis.close();
		String base64CameraRos = new String(Base64.encodeBase64(content));
		
		File workplaneZip = new File("C:/Users/Aristides/Desktop/Six Axis/nodes.zip");
		fis = new FileInputStream(workplaneZip);
		content = new byte[(int) workplaneZip.length()];
		fis.read(content);
		fis.close();
		String base64WorkplaneRos = new String(Base64.encodeBase64(content));
		
		File penJar = new File("C:/Users/Aristides/Desktop/Six Axis/Pen.jar");
		fis = new FileInputStream(penJar);
		content = new byte[(int) penJar.length()];
		fis.read(content);
		fis.close();
		String base64Pen = new String(Base64.encodeBase64(content));
		
		File gripperJar = new File("C:/Users/Aristides/Desktop/Six Axis/Gripper.jar");
		fis = new FileInputStream(gripperJar);
		content = new byte[(int) gripperJar.length()];
		fis.read(content);
		fis.close();
		String base64Gripper = new String(Base64.encodeBase64(content));
		
		File drawJar = new File("C:/Users/Aristides/Desktop/Six Axis/Draw.jar");
		fis = new FileInputStream(drawJar);
		content = new byte[(int) drawJar.length()];
		fis.read(content);
		fis.close();
		String base64Draw = new String(Base64.encodeBase64(content));
		
		File pickAndPlaceJar = new File("C:/Users/Aristides/Desktop/Six Axis/PickAndPlaceWithRotation.jar");
		fis = new FileInputStream(pickAndPlaceJar);
		content = new byte[(int) pickAndPlaceJar.length()];
		fis.read(content);
		fis.close();
		String base64PickAndPlace = new String(Base64.encodeBase64(content));
		
		
		
		/*// gripper
		String moduleA = moduleA_01 + base64DeltaRobotRos + moduleA_02 + base64Module + moduleA_03 + base64Draw + moduleA_04 + base64PickAndPlace + moduleA_05; 
		JsonObject a = new JsonParser().parse(moduleA).getAsJsonObject();
		hal.insertModule(a, a);
		
//		// pen
//		String moduleB = moduleB_01 + base64Pen + moduleB_02; 
//		JsonObject b = new JsonParser().parse(moduleB).getAsJsonObject();
//		hal.insertModule(b, b);
		
		// gripper
		String moduleB = moduleB_01 + base64GripperRos + moduleB_02 + base64Gripper + moduleB_03; 
		JsonObject b = new JsonParser().parse(moduleB).getAsJsonObject();
		hal.insertModule(b, b);
		
		// camera
		String moduleC = moduleC_01 + base64CameraRos + moduleC_02 + base64Pen + moduleC_03;
		JsonObject c = new JsonParser().parse(moduleC).getAsJsonObject();
		hal.insertModule(c, c);
		
		// lens
		String moduleD = moduleD_01 + base64Pen + moduleD_02; 
		JsonObject d = new JsonParser().parse(moduleD).getAsJsonObject();
		hal.insertModule(d, d);
		
		// workplane
		String moduleE = moduleE_01 + base64WorkplaneRos + moduleE_02 + base64Pen + moduleE_03;
		JsonObject e = new JsonParser().parse(moduleE).getAsJsonObject();
		hal.insertModule(e, e);*/

		

		
		
		
		
		
		
		
		JsonObject criteria = new JsonObject();
		JsonObject target = new JsonObject();
		JsonObject targetMove = new JsonObject();
		targetMove.addProperty("x", -10.0);
		targetMove.addProperty("y", -10.0);
		targetMove.addProperty("z", -335.0);
		targetMove.addProperty("maxAcceleration", 2);
		
		targetMove.addProperty("rotationX", 0);
		targetMove.addProperty("rotationY", 0);
		targetMove.addProperty("rotationZ", Math.toRadians(25));
		target.add("move",targetMove);
		target.addProperty("identifier", "Paper");
		
		JsonArray subjects = new JsonArray();
		JsonObject subject = new JsonObject();
		JsonObject subjectMove = new JsonObject();
		subjectMove.addProperty("x", 0.0);
		subjectMove.addProperty("y", 0.0);
		subjectMove.addProperty("z", -360.0);
		
		subjectMove.addProperty("rotationX", 0);
		subjectMove.addProperty("rotationY", 0);
		subjectMove.addProperty("rotationZ", Math.toRadians(25));
		
		
		subject.add("move",subjectMove);
		subject.addProperty("identifier", "Paper");
		subjects.add(subject);
		
		criteria.add("target",target);
		criteria.add("subjects",subjects);
		//criteria.add("subjects", new JsonArray());
		
		
		hal.translateProductStep(new ProductStep("1", criteria, new Service("place")));
		
		/*Service service = new Service("PickAndPlace");
		ProductStep productStep = new ProductStep(0, null, service);
		hal.translateProductStep(productStep);*/
		

	}
	
	@Override
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareStep) {
		// TODO Auto-generated method stub
		System.out.println("Translation finished");
		hardwareSteps.addAll(hardwareStep);// = hardwareStep;
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onIncapableCapabilities(ProductStep productStep) {
		System.out.println("Translation failed because productStep with service " + productStep.getService().getName() + " has no supported capabilities");
	}

	@Override
	public void onProcessStatusChanged(String state, long hardwareStepSerialId,
			Module module, HardwareStep hardwareStep) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onModuleStateChanged(String state, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onModuleModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String getEquipletName() {
		// TODO hardcoded!!!!!!
		return "EQ3";
	}

	@Override
	public void onExecutionFinished() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onEquipletStateChanged(String state, Module module) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onEquipletModeChanged(String mode, Module module) {
		// TODO Auto-generated method stub
		
	}

}
