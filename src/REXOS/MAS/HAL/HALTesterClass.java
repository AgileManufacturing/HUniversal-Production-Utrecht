package HAL;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;

import HAL.listeners.HardwareAbstractionLayerListener;
import HAL.steps.HardwareStep;
import HAL.steps.ProductStep;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public class HALTesterClass implements HardwareAbstractionLayerListener {
	static HALTesterClass htc = new HALTesterClass();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	static BlackboardHandler blackboardUpdated;
	
	// delta robot
	static String moduleA_01 = "{"
			+ "	\"manufacturer\":\"HU\","
			+ "	\"typeNumber\":\"delta_robot_type_A\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"{"
			+ "	\\\"midPointX\\\" : 75.0,"
			+ "	\\\"midPointY\\\" : -200.0,"
			+ "	\\\"midPointZ\\\" : -35.3,"
			+ "	\\\"deltaRobotMeasures\\\" : {"
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
			+ "	\"typeNumber\":\"blue_pen_type_A\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"\","
			+ "		\"rosSoftware\":null,"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleB_02 = "\","
			+ "			\"className\":\"HAL.modules.Pen\""
			+ "		},"
			+ "		\"supportedMutations\": ["
			+ "			\"draw\""
			+ "		],"
			+ "		\"capabilities\":["
			+ "		]"
			+ "	},"
			+ "	\"properties\":\"name\","
			+ "	\"calibrationData\":["
			+ "	],"
			+ "	\"attachedTo\":{"
			+ "		\"manufacturer\":\"HU\","
			+ "		\"typeNumber\":\"delta_robot_type_A\","
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
			+ "			\"command\":\"roslaunch camera.launch {equipletName} {manufacturer} {typeNumber} {serialNumber}\""
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

		File deltaRobotJar = new File("C:/Users/Tommy/Contacts/Desktop/DeltaRobot.jar");
		fis = new FileInputStream(deltaRobotJar);
		content = new byte[(int) deltaRobotJar.length()];
		fis.read(content);
		fis.close();
		String base64Module = new String(Base64.encodeBase64(content));
		
		File deltaRobotZip = new File("C:/Users/Tommy/Contacts/Desktop/nodes.zip");
		fis = new FileInputStream(deltaRobotZip);
		content = new byte[(int) deltaRobotZip.length()];
		fis.read(content);
		fis.close();
		String base64DeltaRobotRos = new String(Base64.encodeBase64(content));
		
		File cameraZip = new File("C:/Users/Tommy/Contacts/Desktop/nodes.zip");
		fis = new FileInputStream(cameraZip);
		content = new byte[(int) cameraZip.length()];
		fis.read(content);
		fis.close();
		String base64CameraRos = new String(Base64.encodeBase64(content));
		
		File penJar = new File("C:/Users/Tommy/Contacts/Desktop/Pen.jar");
		fis = new FileInputStream(penJar);
		content = new byte[(int) penJar.length()];
		fis.read(content);
		fis.close();
		String base64Pen = new String(Base64.encodeBase64(content));
		
		File pickAndPlaceJar = new File("C:/Users/Tommy/Contacts/Desktop/Draw.jar");
		fis = new FileInputStream(pickAndPlaceJar);
		content = new byte[(int) pickAndPlaceJar.length()];
		fis.read(content);
		fis.close();
		String base64Capability = new String(Base64.encodeBase64(content));
		
		// deltarobot
		String moduleA = moduleA_01 + base64DeltaRobotRos + moduleA_02 + base64Module + moduleA_03 + base64Capability + moduleA_04; 
		JsonObject a = new JsonParser().parse(moduleA).getAsJsonObject();
		hal.insertModule(a, a);
		
		// pen
		String moduleB = moduleB_01 + base64Pen + moduleB_02; 
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
		
		
		/*JsonObject tommasgtaylordfuckzooi;
		ModuleIdentifier moduleIdentifier = new ModuleIdentifier("HU", "delta_robot_type_A", "1");
		
		
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);
		tommasgtaylordfuckzooi = hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);
		hal.getModuleFactory().getModuleByIdentifier(moduleIdentifier);*/

		
		JsonObject criteria = new JsonObject();
		JsonObject target = new JsonObject();
		JsonObject targetMove = new JsonObject();
		targetMove.addProperty("x", 0.0);
		targetMove.addProperty("y", 0.0);
		targetMove.addProperty("z", 20.0);
		target.add("move",targetMove);
		target.addProperty("identifier", "Paper");
		
		JsonArray subjects = new JsonArray();
		JsonObject subject = new JsonObject();
		JsonObject subjectMove = new JsonObject();
		subjectMove.addProperty("x", -3.0);
		subjectMove.addProperty("y", 0.0);
		subjectMove.addProperty("z", 0.0);
		subject.add("move",subjectMove);
		subjects.add(subject);
		
		criteria.add("target",target);
		//criteria.add("subjects",subjects);
		criteria.add("subjects", new JsonArray());
		
		
		hal.translateProductStep(
				new ProductStep("1", criteria, new Service("draw")));
		
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
		return "EQ1";
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
