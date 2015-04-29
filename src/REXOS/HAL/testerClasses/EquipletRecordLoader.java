package HAL.testerClasses;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.dataTypes.GazeboModel;
import HAL.dataTypes.JavaSoftware;
import HAL.dataTypes.RosSoftware;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;

public class EquipletRecordLoader {
	
	static String equipletName = "EQ2";
	
	public static void main(String[] args) throws JSONException, IOException {
		if(args.length >= 1) {
			equipletName = args[0];
		}
		System.out.println("Inserting equiplet " + equipletName);
		
		KnowledgeDBClient client = new KnowledgeDBClient();
		
		JSONObject javaSoftwareJson = new JSONObject();
		javaSoftwareJson.put(JavaSoftware.BUILD_NUMBER, 1);
		// TODO EQ has no java software ATM
		javaSoftwareJson.put(JavaSoftware.JAR_FILE, "");
		javaSoftwareJson.put(JavaSoftware.CLASS_NAME, "");
		JavaSoftware javaSoftware = JavaSoftware.deSerialize(javaSoftwareJson);
		
		JSONObject gazeboModelJson = new JSONObject();
		gazeboModelJson.put(GazeboModel.BUILD_NUMBER, 1);
		File modelZipFile = new File("generatedOutput/models/equiplet.zip");
		FileInputStream modelZipFileStream = new FileInputStream(modelZipFile);
		byte[] modelZipFileContent = new byte[(int) modelZipFile.length()];
		modelZipFileStream.read(modelZipFileContent);
		modelZipFileStream.close();
		gazeboModelJson.put(GazeboModel.ZIP_FILE, new String(Base64.encodeBase64(modelZipFileContent)));
		gazeboModelJson.put(GazeboModel.SDF_FILE_NAME, "model.sdf");
		gazeboModelJson.put(GazeboModel.PARENT_LINK, "base");
		gazeboModelJson.put(GazeboModel.CHILD_LINK, "base");
		gazeboModelJson.put(GazeboModel.CHILD_LINK_OFFSET_X, -226.88);
		gazeboModelJson.put(GazeboModel.CHILD_LINK_OFFSET_Y, -36.00);
		gazeboModelJson.put(GazeboModel.CHILD_LINK_OFFSET_Z, 1141.97);
		GazeboModel gazeboModel = GazeboModel.deSerialize(gazeboModelJson);
		
		JSONObject rosSoftwareJson = new JSONObject();
		rosSoftwareJson.put(RosSoftware.BUILD_NUMBER, 1);
		File nodeZipFile = new File("generatedOutput/nodes/equiplet.zip");
		FileInputStream nodeZipFileStream = new FileInputStream(nodeZipFile);
		byte[] nodeZipFileContent = new byte[(int) nodeZipFile.length()];
		nodeZipFileStream.read(nodeZipFileContent);
		nodeZipFileStream.close();
		rosSoftwareJson.put(RosSoftware.ROS_FILE, new String(Base64.encodeBase64(nodeZipFileContent)));
		rosSoftwareJson.put(RosSoftware.COMMAND, "rosrun equiplet_node equiplet_node {equipletName}");
		RosSoftware rosSoftware = RosSoftware.deSerialize(rosSoftwareJson);
		
		javaSoftware.insertIntoDatabase(client);
		gazeboModel.insertIntoDatabase(client);
		rosSoftware.insertIntoDatabase(client);
		
		client.executeUpdateQuery(
					"insert into Equiplet \n" + 
					"(name, mountPointsX, mountPointsY, mountPointDistanceX, mountPointDistanceY, rosSoftware, masSoftware, gazeboModel) \n" + 
					"values(?, ?, ?, ?, ?, ?, ?, ?);",
				equipletName, 10, 18, 50.0, 50.0, rosSoftware.id, javaSoftware.id, gazeboModel.id);
		
		System.out.println("Done");
	}
}
