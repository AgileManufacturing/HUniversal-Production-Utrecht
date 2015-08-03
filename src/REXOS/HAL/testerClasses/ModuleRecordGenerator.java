package HAL.testerClasses;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;

import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import util.log.LogLevel;
import util.log.Logger;
import HAL.dataTypes.ModuleIdentifier;
import HAL.dataTypes.ModuleTypeIdentifier;
import HAL.dataTypes.SerializedCapability;
import HAL.dataTypes.StaticSettings;

public class ModuleRecordGenerator {
	public static final String srcDirectory = "src/REXOS/modules/";
	public static final String dstDirectory = "generatedOutput/staticSettings/";
	public static final String rosDirectory = "generatedOutput/nodes/";
	public static final String halModuleDirectory = "generatedOutput/HAL/modules/";
	public static final String halCapabilityDirectory = "generatedOutput/HAL/capabilities/";
	public static final String modelModuleDirectory = "generatedOutput/models/modules/";
	
	public static void main(String[] args) throws Exception {
		File srcFile = new File(srcDirectory);
		for (File manufacturer : srcFile.listFiles()) {
			if(manufacturer.isDirectory() == false) continue;
			if(manufacturer.getName().startsWith("_")) continue;
			
			for (File typeNumber : manufacturer.listFiles()) {
				if(typeNumber.isDirectory() == false) continue;
				if(typeNumber.getName().startsWith("_")) continue;
				
				for (File serialNumber : typeNumber.listFiles()) {
					if(serialNumber.isDirectory() == false) continue;
					if(serialNumber.getName().startsWith("_")) continue;
					
					ModuleIdentifier moduleIdentifier = new ModuleIdentifier(manufacturer.getName(), typeNumber.getName(), serialNumber.getName());
					Logger.log(LogLevel.DEBUG, "generating static settings", moduleIdentifier);
					generateStaticSettings(moduleIdentifier);
				}
			}
		}
	}
	
	private static void generateStaticSettings(ModuleIdentifier moduleIdentifier) throws JSONException, ParseException, IOException {
		// combine path components to get <manufacturer>/<typeNumber>/<serialNumber>/
		File srcPath = new File(srcDirectory);
		srcPath = new File(srcPath, moduleIdentifier.manufacturer);
		srcPath = new File(srcPath, moduleIdentifier.typeNumber);
		srcPath = new File(srcPath, moduleIdentifier.serialNumber);
		File srcStaticSettingsFile = new File(srcPath, "staticSettings.json");
		
		FileReader srcStaticSettingsFileReader = new FileReader(srcStaticSettingsFile);
		JSONTokener srcStaticSettingsJsonTokener = new JSONTokener(srcStaticSettingsFileReader);
		JSONObject srcStaticSettingsJsonObject = new JSONObject(srcStaticSettingsJsonTokener);
		StaticSettings staticSettings = StaticSettings.deSerialize(srcStaticSettingsJsonObject);
		
		// ROS software
		if(staticSettings.moduleType.rosSoftware != null) {
			File rosFileFile = getRosFile(moduleIdentifier); 
			byte[] rosFileContent = new byte[(int) rosFileFile.length()];
			FileInputStream rosFileFileInputStream = new FileInputStream(rosFileFile);
			rosFileFileInputStream.read(rosFileContent);
			staticSettings.moduleType.rosSoftware.rosFile = rosFileContent;
			rosFileFileInputStream.close();
		}
		// HAL software
		if(staticSettings.moduleType.halSoftware != null) {
			File halFileFile = getHalModuleFile(moduleIdentifier); 
			byte[] halFileContent = new byte[(int) halFileFile.length()];
			FileInputStream halFileFileInputStream = new FileInputStream(halFileFile);
			halFileFileInputStream.read(halFileContent);
			staticSettings.moduleType.halSoftware.jarFile = halFileContent;
			halFileFileInputStream.close();
		}
		// model
		if(staticSettings.moduleType.gazeboModel != null) {
			File modelFileFile = getModelFile(moduleIdentifier); 
			byte[] modelFileContent = new byte[(int) modelFileFile.length()];
			FileInputStream modelFileFileInputStream = new FileInputStream(modelFileFile);
			modelFileFileInputStream.read(modelFileContent);
			staticSettings.moduleType.gazeboModel.zipFile = modelFileContent;
			modelFileFileInputStream.close();
		}
		
		// capabilities
		for (SerializedCapability capability : staticSettings.moduleType.capabilities) {
			File halFileFile = getHalCapabilityFile(capability);
			byte[] halFileContent = new byte[(int) halFileFile.length()];
			FileInputStream halFileFileInputStream = new FileInputStream(halFileFile);
			halFileFileInputStream.read(halFileContent);
			capability.halSoftware.jarFile = halFileContent;
			halFileFileInputStream.close();
		}
		
		File dstPath = new File(dstDirectory);
		dstPath = new File(dstPath, moduleIdentifier.manufacturer);
		dstPath = new File(dstPath, moduleIdentifier.typeNumber);
		dstPath = new File(dstPath, moduleIdentifier.serialNumber);
		File dstStaticSettingsFile = new File(dstPath, "staticSettings.json");
		dstStaticSettingsFile.mkdirs();
		if(dstStaticSettingsFile.exists()) dstStaticSettingsFile.delete();
		dstStaticSettingsFile.createNewFile();
		
		JSONObject dstStaticSettings = staticSettings.serialize();
		FileWriter dstFileWriter = new FileWriter(dstStaticSettingsFile);
		dstFileWriter.write(dstStaticSettings.toString());
		dstFileWriter.close();
	}

	private static File getRosFile(ModuleTypeIdentifier moduleTypeIdentifier) {
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("delta_robot_type_A")) {
			return new File(rosDirectory, "delta_robot.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("delta_robot_type_B")) {
			return new File(rosDirectory, "delta_robot.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("gripper_type_A")) {
			return new File(rosDirectory, "gripper.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("gripper_type_B")) {
			return new File(rosDirectory, "gripper.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("stewart_gough_type_A")) {
			return new File(rosDirectory, "stewart_gough.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("workplane_type_A")) {
			return new File(rosDirectory, "workplane.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("The_Imaging_Source_Europe_GmbH") && moduleTypeIdentifier.typeNumber.equals("DFK_22AUC03")) {
			return new File(rosDirectory, "huniversal_camera.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("dummy_module_type_A")) {
			return new File(rosDirectory, "_testing.zip");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("dummy_module_type_B")) {
			return new File(rosDirectory, "_testing.zip");
		}
		throw new RuntimeException("Unknown module type");
	}
	
	private static File getHalModuleFile(ModuleTypeIdentifier moduleTypeIdentifier) {
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("delta_robot_type_A")) {
			return new File(halModuleDirectory, "DeltaRobot.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("delta_robot_type_B")) {
			return new File(halModuleDirectory, "DeltaRobot.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("gripper_type_A")) {
			return new File(halModuleDirectory, "Gripper.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("gripper_type_B")) {
			return new File(halModuleDirectory, "Gripper.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("pen_type_A")) {
			return new File(halModuleDirectory, "Pen.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("stewart_gough_type_A")) {
			return new File(halModuleDirectory, "StewartGough.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("workplane_type_A")) {
			return new File(halModuleDirectory, "Workplane.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("dummy_module_type_A")) {
			return new File(halModuleDirectory, "DummyModuleA.jar");
		}
		if(moduleTypeIdentifier.manufacturer.equals("HU") && moduleTypeIdentifier.typeNumber.equals("dummy_module_type_B")) {
			return new File(halModuleDirectory, "DummyModuleB.jar");
		}
		throw new RuntimeException("Unknown module type");
	}

	private static File getModelFile(ModuleTypeIdentifier moduleTypeIdentifier) {
		File modelZip = new File(modelModuleDirectory);
		modelZip = new File(modelZip, moduleTypeIdentifier.manufacturer);
		modelZip = new File(modelZip, moduleTypeIdentifier.typeNumber + ".zip");
		return modelZip;
	}
	
	private static File getHalCapabilityFile(SerializedCapability capability) {
		File capabilityJar = new File(halCapabilityDirectory);
		capabilityJar = new File(capabilityJar, capability.name + ".jar");
		return capabilityJar;
	}

}
