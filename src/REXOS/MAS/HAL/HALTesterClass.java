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
	
	static boolean translationFinished = false;
	
	double stepX = 30.0;
	boolean dirLeft = false;
	
	static ArrayList<ArrayList<HardwareStep>> translatedProductSteps = new ArrayList<ArrayList<HardwareStep>>(); 
	
	static String moduleA_01 = "{"
			+ "	\"manufacturer\":\"HU\","
			+ "	\"typeNumber\":\"delta_robot_type_A\","
			+ "	\"serialNumber\":\"1\","
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
			+ "				\"name\":\"Draw\","
			+ "				\"treeNumber\":1,"
			+ "				\"halSoftware\":{"
			+ "					\"buildNumber\":1,"
			+ "					\"jarFile\": \"";
	static String moduleA_03 = "\","
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
			+ "	\"properties\":\"name\","
			+ "	\"calibrationData\":["
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
			+ "			\"className\":\"moduleClass\""
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
		

		
		FileInputStream fis;
		byte[] content;

		File deltaRobotJar = new File("/home/t/Desktop/DeltaRobot.jar");
		fis = new FileInputStream(deltaRobotJar);
		content = new byte[(int) deltaRobotJar.length()];
		fis.read(content);
		fis.close();
		String base64Module = new String(Base64.encodeBase64(content));
		System.out.println("delta robot module: " + base64Module);
		
		File gripperJar = new File("/home/t/Desktop/Pen.jar");
		fis = new FileInputStream(gripperJar);
		content = new byte[(int) gripperJar.length()];
		fis.read(content);
		fis.close();
		String base64Gripper = new String(Base64.encodeBase64(content));
		System.out.println("gripper module: " + base64Gripper);
		
		File pickAndPlaceJar = new File("/home/t/Desktop/Draw.jar");
		fis = new FileInputStream(pickAndPlaceJar);
		content = new byte[(int) pickAndPlaceJar.length()];
		fis.read(content);
		fis.close();
		String base64Capability = new String(Base64.encodeBase64(content));
		System.out.println("pick and place capability: " + base64Capability);
		
		String moduleA = moduleA_01 + base64Module + moduleA_02 + base64Capability + moduleA_03; 
		JsonObject a = new JsonParser().parse(moduleA).getAsJsonObject();
		hal.insertModule(a, a);
		
		String moduleB = moduleB_01 + base64Gripper + moduleB_02; 
		JsonObject b = new JsonParser().parse(moduleB).getAsJsonObject();
		hal.insertModule(b, b);
		
		hal.getBottomModules();
		

		JsonObject criteria = new JsonObject();
		JsonObject target = new JsonObject();
		JsonObject targetMove = new JsonObject();
		targetMove.addProperty("x", 0.0);
		targetMove.addProperty("y", 0.0);
		targetMove.addProperty("z", -320.0);
		target.add("move",targetMove);
		target.addProperty("identifier", "Paper");
		
		criteria.add("target",target);
		criteria.add("subjects", new JsonArray()); 
		hal.translateProductStep(new ProductStep(1, criteria, new Service("draw")));
		
		System.out.println("While loop starting");
		/*Service service = new Service("PickAndPlace");
		ProductStep productStep = new ProductStep(0, null, service);
		hal.translateProductStep(productStep);*/
		while(true){
			Thread.sleep(1);
			//System.out.println("While loop");
			if (translationFinished){
				// TODO Auto-generated method stub
				//hardwareSteps.addAll(hardwareStep);// = hardwareStep;
				//translatedProductSteps.add(hardwareSteps);
				
				System.out.println("translation result " + hardwareSteps);
				
				hal.executeHardwareSteps(hardwareSteps);
				translationFinished = false;
			}
		}

	}

	@Override
	public void onProcessStateChanged(String state, long hardwareStepSerialId, Module module, HardwareStep hardwareStep) {
		System.out.println("ik ben klaar!!!!!!!!!!!!!!!!!!!!");
		
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
		translationFinished = true;
		hardwareSteps = (ArrayList<HardwareStep>) hardwareStep.clone();
		System.out.println("onTranslationFinished");
	}

	@Override
	public void onIncapableCapabilities(ProductStep productStep) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onExecutionFinished() {
		stepX = -stepX;
		// TODO Auto-generated method stub
		JsonObject criterias = new JsonObject();
		JsonObject targets = new JsonObject();
		JsonObject targetMoves = new JsonObject();
		targetMoves.addProperty("x", stepX);
		targetMoves.addProperty("y", 0.0);
		targetMoves.addProperty("z", -320.0);
		targets.add("move",targetMoves);
		targets.addProperty("identifier", "Paper");
		
		criterias.add("target",targets);
		criterias.add("subjects", new JsonArray()); 
		System.out.println("target Moves: " + targetMoves);
		new Exception().printStackTrace(System.out);
		hal.translateProductStep(new ProductStep(2, criterias, new Service("draw")));
	}

}
