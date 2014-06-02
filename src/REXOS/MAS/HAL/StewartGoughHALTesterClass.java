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
	// six_axis robot
/*		static String moduleA_01 = "{"
				+ "	\"manufacturer\":\"HU\","
				+ "	\"typeNumber\":\"six_axis_type_A\","
				+ "	\"serialNumber\":\"1\","
				+ "	\"moduleType\":{"
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
				+ "	\"moduleType\":{"
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
				+ "	\"moduleType\":{"
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
				+ "	\"moduleType\":{"
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
				+ "}";*/
	

	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {
		System.out.println("Starting");
		
		// TODO Auto-generated method stub
		hal = new HardwareAbstractionLayer(htc);
		System.out.println("done hal");


		JsonObject criteria = new JsonObject();
		JsonObject target = new JsonObject();
		JsonObject targetMove = new JsonObject();
		targetMove.addProperty("x", -3.0);
		targetMove.addProperty("y", 5.0);
		targetMove.addProperty("z", -410.0);
		target.add("move",targetMove);
		target.addProperty("identifier", "GC4x4MB_1");
		
		JsonArray subjects = new JsonArray();
		JsonObject subject = new JsonObject();
		JsonObject subjectMove = new JsonObject();
		subjectMove.addProperty("x", -3.0);
		subjectMove.addProperty("y", 3.0);
		subjectMove.addProperty("z", 410.0);
		subject.add("move",subjectMove);
		subjects.add(subject);
		
		criteria.add("target",target);
		criteria.add("subjects", new JsonArray());
		
		
		hal.translateProductStep(new ProductStep("1", criteria, new Service("place")));

		

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
	public void onProcessStateChanged(String state, long hardwareStepSerialId,
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
		return "EQ2";
	}

	@Override
	public void onExecutionFinished() {
		// TODO Auto-generated method stub
		
	}

}
