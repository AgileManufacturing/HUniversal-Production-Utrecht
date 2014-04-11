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
			+ "	\"manufacturer\":\"manA\","
			+ "	\"typeNumber\":\"typeA\","
			+ "	\"serialNumber\":\"serA\","
			+ "	\"type\":{"
			+ "		\"properties\":\"hoi!\","
			+ "		\"rosSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"rosFile\": \"\","
			+ "			\"className\":\"moduleClass\""
			+ "		},"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleA_02 = "\","
			+ "			\"className\":\"HAL.modules.DeltaRobot\""
			+ "		},"
			+ "		\"supportedMutations\": ["
			+ "			\"move\""
			+ "		],"
			+ "		\"capabilities\":["
			+ "			{"
			+ "				\"name\":\"PickAndPlace\","
			+ "				\"treeNumber\":1,"
			+ "				\"halSoftware\":{"
			+ "					\"buildNumber\":1,"
			+ "					\"jarFile\": \"";
	static String moduleA_03 = "\","
			+ "					\"className\":\"HAL.capabilities.PickAndPlace\""
			+ "				},"
			+ "				\"requiredMutationsTrees\":["
			+ "					{"
			+ "						\"treeNumber\":1,"
			+ "						\"mutations\":["
			+ "							\"move\""
			+ "						]"
			+ "					}"
			+ "				],"
			+ "				\"services\":["
			+ "					\"place\""
			+ "				]"
			+ "			}"
			+ "		]"
			+ "	},"
			+ "	\"properties\":\"name\","
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
			+ "\"mountPointX\":1,"
			+ "\"mountPointY\":1"
			+ "}";
	static String moduleB_01 = "{"
			+ "	\"manufacturer\":\"manA\","
			+ "	\"typeNumber\":\"typeB\","
			+ "	\"serialNumber\":\"serA\","
			+ "	\"type\":{"
			+ "		\"properties\":\"hoi!\","
			+ "		\"rosSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"rosFile\": \"\","
			+ "			\"className\":\"moduleClass\""
			+ "		},"
			+ "		\"halSoftware\":{"
			+ "			\"buildNumber\":1,"
			+ "			\"jarFile\": \"";
	static String moduleB_02 = "\","
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
			+ "		\"manufacturer\":\"manA\","
			+ "		\"typeNumber\":\"typeA\","
			+ "		\"serialNumber\":\"serA\""
			+ "	},"
			+ "\"mountPointX\":null,"
			+ "\"mountPointY\":null"
			+ "}";
			
	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		// TODO Auto-generated method stub
		hal = new HardwareAbstractionLayer(htc);
		BlackboardListener blackboardListener = new BlackboardListener() {
			
			@Override
			public void onProcessStatusChanged(String status) {
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
			public void OnEquipleStateChanged(String id, String state) {
				// TODO Auto-generated method stub
				System.out.println("New State of EQ "+id + " = "+ state);
				
			}
			
			@Override
			public void OnEquipleModeChanged(String id, String mode) {
				// TODO Auto-generated method stub
				System.out.println("New mode of EQ "+id + " = "+ mode);
				
			}
		};
		
		blackboardUpdated = new BlackboardUpdated();
		blackboardUpdated.setOnBlackBoardUpdatedListener(blackboardListener);
//		while(true){
//			System.out.println("Waiting on status change");
//			Thread.sleep(4000);
//		}
		
		
		
		
//		FileInputStream fis;
//		byte[] content;
//
//		File deltaRobotJar = new File("/home/t/Desktop/DeltaRobot.jar");
//		fis = new FileInputStream(deltaRobotJar);
//		content = new byte[(int) deltaRobotJar.length()];
//		fis.read(content);
//		fis.close();
//		String base64Module = new String(Base64.encodeBase64(content));
//		
//		File gripperJar = new File("/home/t/Desktop/Gripper.jar");
//		fis = new FileInputStream(gripperJar);
//		content = new byte[(int) gripperJar.length()];
//		fis.read(content);
//		fis.close();
//		String base64Gripper = new String(Base64.encodeBase64(content));
//		
//		File pickAndPlaceJar = new File("/home/t/Desktop/PickAndPlace.jar");
//		fis = new FileInputStream(pickAndPlaceJar);
//		content = new byte[(int) pickAndPlaceJar.length()];
//		fis.read(content);
//		fis.close();
//		String base64Capability = new String(Base64.encodeBase64(content));
//		
//		String moduleA = moduleA_01 + base64Module + moduleA_02 + base64Capability + moduleA_03; 
//		JsonObject a = new JsonParser().parse(moduleA).getAsJsonObject();
//		hal.insertModule(a, a);
//		
//		String moduleB = moduleB_01 + base64Gripper + moduleB_02; 
//		JsonObject b = new JsonParser().parse(moduleB).getAsJsonObject();
//		hal.insertModule(b, b);
//		
//		hal.getBottomModules();
//		
//		
//		JsonObject criteria = new JsonObject();
//		JsonObject target = new JsonObject();
//		JsonObject targetMove = new JsonObject();
//		targetMove.addProperty("x", 3.0);
//		targetMove.addProperty("y", 0.0);
//		targetMove.addProperty("z", 3.0);
//		target.add("move",targetMove);
//		
//		JsonArray subjects = new JsonArray();
//		JsonObject subject = new JsonObject();
//		JsonObject subjectMove = new JsonObject();
//		subjectMove.addProperty("x", -3.0);
//		subjectMove.addProperty("y", 0.0);
//		subjectMove.addProperty("z", -3.0);
//		subject.add("move",subjectMove);
//		subjects.add(subject);
//		
//		criteria.add("target",target);
//		criteria.add("subjects",subjects);
//		
//		
//		hal.translateProductStep(
//				new ProductStep(1, criteria, new Service("place")));
		
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
		//hardwareSteps.addAll(hardwareStep);// = hardwareStep;
		//hal.executeHardwareSteps(hardwareSteps);
	}

	@Override
	public void onIncapableCapabilities(ProductStep productStep) {
		// TODO Auto-generated method stub
		
	}

}
