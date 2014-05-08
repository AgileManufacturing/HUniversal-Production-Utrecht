package HAL;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.ArrayList;

import org.apache.commons.codec.binary.Base64;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import libraries.knowledgedb_client.KnowledgeException;
import libraries.utillities.log.LogLevel;
import libraries.utillities.log.Logger;
import HAL.listeners.BlackboardListener;
import HAL.listeners.HardwareAbstractionLayerListener;

public class HALTesterClass implements HardwareAbstractionLayerListener {
	static HALTesterClass htc = new HALTesterClass();
	static ArrayList<HardwareStep> hardwareSteps = new ArrayList<HardwareStep>();
	static HardwareAbstractionLayer hal;
	static BlackboardUpdated blackboardUpdated;
	
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
			+ "}","
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
			+ "	\"modbusIp\" : \"192.168.0.22\","
			+ "	\"modbusPort\" : 502"
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
	static String moduleB_01 = "{"
			+ "	\"manufacturer\":\"HU\","
			+ "	\"typeNumber\":\"blue_pen_type_A\","
			+ "	\"serialNumber\":\"1\","
			+ "	\"type\":{"
			+ "		\"properties\":\"hoi!\","
			+ "		\"rosSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"rosFile\": \"\","
			+ "			\"command\":\"moduleClass\""
			+ "		},"
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
			
	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		System.out.println("Starting");
		
		// TODO Auto-generated method stub
		hal = new HardwareAbstractionLayer(htc);
		BlackboardListener blackboardListener = new BlackboardListener() {
			
			@Override
			public void onProcessStateChanged(String status) {
				// TODO Auto-generated method stub
				System.out.println("New Process Status = "+ status);
				
			}
			
			@Override
			public void onModuleStateChanged(String state) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void onModuleModeChanged(String mode) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void OnEquipletStateChanged(String id, String state) {
				// TODO Auto-generated method stub
				System.out.println("New State of EQ "+id + " = "+ state);
				
			}
			
			@Override
			public void OnEquipletModeChanged(String id, String mode) {
				// TODO Auto-generated method stub
				System.out.println("New mode of EQ "+id + " = "+ mode);
				
			}

			@Override
			public void OnEquipletIpChanged(String ip) {
				// TODO Auto-generated method stub
				
			}
		};
		
		/*blackboardUpdated = new BlackboardUpdated();
		blackboardUpdated.setOnBlackBoardUpdatedListener(blackboardListener);*/

		
		FileInputStream fis;
		byte[] content;

		File deltaRobotJar = new File("/home/t/Desktop/DeltaRobot.jar");
		fis = new FileInputStream(deltaRobotJar);
		content = new byte[(int) deltaRobotJar.length()];
		fis.read(content);
		fis.close();
		String base64Module = new String(Base64.encodeBase64(content));
		//System.out.println("delta robot module: " + base64Module);
		
		File deltaRobotZip = new File("/home/t/Desktop/nodes.zip");
		fis = new FileInputStream(deltaRobotZip);
		content = new byte[(int) deltaRobotZip.length()];
		fis.read(content);
		fis.close();
		String base64ModuleRos = new String(Base64.encodeBase64(content));
		//System.out.println("delta robot module zip: " + base64ModuleRos);
		
		File gripperJar = new File("/home/t/Desktop/Pen.jar");
		fis = new FileInputStream(gripperJar);
		content = new byte[(int) gripperJar.length()];
		fis.read(content);
		fis.close();
		String base64Gripper = new String(Base64.encodeBase64(content));
		//System.out.println("gripper module: " + base64Gripper);
		
		File pickAndPlaceJar = new File("/home/t/Desktop/Draw.jar");
		fis = new FileInputStream(pickAndPlaceJar);
		content = new byte[(int) pickAndPlaceJar.length()];
		fis.read(content);
		fis.close();
		String base64Capability = new String(Base64.encodeBase64(content));
		//System.out.println("pick and place capability: " + base64Capability);
		
		String moduleA = moduleA_01 + base64ModuleRos + moduleA_02 + base64Module + moduleA_03 + base64Capability + moduleA_04; 
		JsonObject a = new JsonParser().parse(moduleA).getAsJsonObject();
		hal.insertModule(a, a);
		
		String moduleB = moduleB_01 + base64Gripper + moduleB_02; 
		JsonObject b = new JsonParser().parse(moduleB).getAsJsonObject();
		//hal.insertModule(b, b);
		
		hal.getBottomModules();
		
		ModuleIdentifier moduleIdentifier = new ModuleIdentifier("HU", "delta_robot_type_A", "1");
		/*JsonObject tommasgtaylordfuckzooi =hal.deleteModule(moduleIdentifier);
		hal.insertModule(tommasgtaylordfuckzooi, a);*/

		/*
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
				new ProductStep(1, criteria, new Service("draw")));
		
		/*Service service = new Service("PickAndPlace");
		ProductStep productStep = new ProductStep(0, null, service);
		hal.translateProductStep(productStep);*/
		

	}

	@Override
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module, HardwareStep hardwareStep) {
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
	public void onTranslationFinished(ProductStep productStep, ArrayList<HardwareStep> hardwareStep) {
		// TODO Auto-generated method stub
		System.out.println("Translation finished");
		hardwareSteps.addAll(hardwareStep);// = hardwareStep;
		hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onIncapableCapabilities(ProductStep productStep) {
		// TODO Auto-generated method stub
		
	}

}
