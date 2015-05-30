package HAL.testerClasses;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;

import HAL.dataTypes.GazeboCollision;
import HAL.dataTypes.GazeboModel;
import HAL.dataTypes.PartType;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;

public class PartRecordLoader {
	
	static String partName = "GC4x4MB_1";
	static String partTypeNumber = "GC4x4MB";
	static String parentPartName = null;
	static String partProperties = "{}";
	static String partTypeProperties = "{}";
	static double positionX = 0;
	static double positionY = 0;
	static double positionZ = 0;
	static double rotationX = 0;
	static double rotationY = 0;
	static double rotationZ = 0;
	
	public static void main(String[] args) throws JSONException, IOException {
		if(args.length >= 9) {
			positionX = Double.parseDouble(args[3]);
			positionY = Double.parseDouble(args[4]);
			positionZ = Double.parseDouble(args[5]);
			rotationX = Double.parseDouble(args[6]);
			rotationY = Double.parseDouble(args[7]);
			rotationZ = Double.parseDouble(args[8]);
		}
		if(args.length >= 3) {
			parentPartName = args[2];
		}
		if(args.length >= 2) {
			partTypeNumber = args[1];
		}
		if(args.length >= 1) {
			partName = args[0];
		} 
		System.out.println("Inserting part " + partName);
		
		KnowledgeDBClient client = new KnowledgeDBClient();
		
		JSONObject gazeboModelJson = new JSONObject();
		gazeboModelJson.put(GazeboModel.BUILD_NUMBER, 1);
		File modelZipFile = new File("generatedOutput/models/" + partTypeNumber + ".zip");
		FileInputStream modelZipFileStream = new FileInputStream(modelZipFile);
		byte[] modelZipFileContent = new byte[(int) modelZipFile.length()];
		modelZipFileStream.read(modelZipFileContent);
		modelZipFileStream.close();
		gazeboModelJson.put(GazeboModel.ZIP_FILE, new String(Base64.encodeBase64(modelZipFileContent)));
		gazeboModelJson.put(GazeboModel.SDF_FILE_NAME, "model.sdf");
		gazeboModelJson.put(GazeboModel.PARENT_LINK, "base");
		gazeboModelJson.put(GazeboModel.CHILD_LINK, "base");
		gazeboModelJson.put(GazeboModel.CHILD_LINK_OFFSET_X, 0);
		gazeboModelJson.put(GazeboModel.CHILD_LINK_OFFSET_Y, 0);
		gazeboModelJson.put(GazeboModel.CHILD_LINK_OFFSET_Z, 0);
		
		JSONArray gazeboCollisions = new JSONArray();
		// TODO remove hardcoding
		if(partTypeNumber.equals("GC4x4MB")) {
			// we are dealing with a crate
			JSONObject gazeboCollisionJson = new JSONObject();
			gazeboCollisionJson.put(GazeboCollision.LINK_NAME, "base");
			gazeboCollisionJson.put(GazeboCollision.COLLISION_NAME, "collision");
			gazeboCollisionJson.put(GazeboCollision.MAX_FORCE, 20.0);
			gazeboCollisionJson.put(GazeboCollision.MAX_TORQUE, 20.0);
			gazeboCollisionJson.put(GazeboCollision.MAY_HAVE_CONTACT_WITH_CHILD_MODULES, true);
			gazeboCollisions.put(gazeboCollisionJson);
		} else if(partTypeNumber.equals("chessboard")) {
			// we are dealing with a chessboard
			JSONObject gazeboCollisionJson = new JSONObject();
			gazeboCollisionJson.put(GazeboCollision.LINK_NAME, "base");
			gazeboCollisionJson.put(GazeboCollision.COLLISION_NAME, "collision");
			gazeboCollisionJson.put(GazeboCollision.MAX_FORCE, 20.0);
			gazeboCollisionJson.put(GazeboCollision.MAX_TORQUE, 20.0);
			gazeboCollisionJson.put(GazeboCollision.MAY_HAVE_CONTACT_WITH_CHILD_MODULES, true);
			gazeboCollisions.put(gazeboCollisionJson);
		} else {
			// we are dealing with a ball
			JSONObject gazeboCollisionJson = new JSONObject();
			gazeboCollisionJson.put(GazeboCollision.LINK_NAME, "base");
			gazeboCollisionJson.put(GazeboCollision.COLLISION_NAME, "collision");
			gazeboCollisionJson.put(GazeboCollision.MAX_FORCE, 20.0);
			gazeboCollisionJson.put(GazeboCollision.MAX_TORQUE, 20.0);
			gazeboCollisionJson.put(GazeboCollision.MAY_HAVE_CONTACT_WITH_CHILD_MODULES, true);
			gazeboCollisions.put(gazeboCollisionJson);
		}
		gazeboModelJson.put(GazeboModel.COLLISIONS, gazeboCollisions);
		
		
		JSONObject partTypeJson = new JSONObject();
		partTypeJson.put(PartType.TYPE_NUMBER, partTypeNumber);
		JSONTokener tokener = new JSONTokener(partTypeProperties);
		partTypeJson.put(PartType.PROPERTIES, new JSONObject(tokener));
		partTypeJson.put(PartType.GAZEBO_MODEL, gazeboModelJson);
		
		PartType partType = PartType.deSerialize(partTypeJson);
		partType.insertIntoDatabase(client);
		
		File qrCodeFile = new File("generatedOutput/models/_qrCodes/" + partName + ".png");
		byte[] qrCodeFileContent = null;
		if(qrCodeFile.exists() == true) {
			FileInputStream qrCodeFileStream = new FileInputStream(qrCodeFile);
			qrCodeFileContent = new byte[(int) qrCodeFile.length()];
			qrCodeFileStream.read(qrCodeFileContent);
			qrCodeFileStream.close();
		}
		
		if(parentPartName == null) {
			client.executeUpdateQuery(
					"INSERT INTO Part \n" +
					"( \n" + 
					"	partName, partType, attachedToLeft, attachedToRight, \n" +
					"	positionX, positionY, positionZ, rotationX, rotationY, rotationZ, \n" +
					"	partProperties, qrCode \n" +
					") \n" +
					"VALUES (?, ?, (\n" +
					"	IFNULL( ( \n" +
					"		SELECT max(attachedToRight) + 1 \n" +
					"		FROM (SELECT * FROM Part) AS tbl1 \n" +
					"	), ( \n" +
					"		1 \n" +
					"	) ) \n" +
					"), ( \n" +
					"	IFNULL( ( \n" +
					"		SELECT max(attachedToRight) + 2 \n" +
					"		FROM (SELECT * FROM Part) AS tbl2 \n" +
					"	), ( \n" +
					"		2 \n" +
					"	) ) \n" +
					"), ?, ?, ?, ?, ?, ?, ?, ?);",
					partName, partType.partTypeNumber,
					positionX, positionY, positionZ, rotationX, rotationY, rotationZ, 
					partProperties, qrCodeFileContent);
		} else {
			client.executeUpdateQuery( 
					"UPDATE Part \n" +
					"SET attachedToLeft = attachedToLeft + 2 \n" +
					"WHERE attachedToLeft > ( \n" +
					"	SELECT attachedToLeft \n" +
					"	FROM (SELECT * FROM Part) AS tbl1 \n" +
					"	WHERE partName = ? \n" + 
					");", 
					parentPartName);
			client.executeUpdateQuery( 
					"UPDATE Part \n" +
					"SET attachedToRight = attachedToRight + 2 \n" +
					"WHERE attachedToRight >= ( \n" +
					"	SELECT attachedToLeft \n" +
					"	FROM (SELECT * FROM Part) AS tbl1 \n" +
					"	WHERE partName = ? \n" + 
					");", 
					parentPartName);
			client.executeUpdateQuery(
					"INSERT INTO Part \n" +
					"( \n" + 
					"	partName, partType, attachedToLeft, attachedToRight, \n" +
					"	positionX, positionY, positionZ, rotationX, rotationY, rotationZ, \n" +
					"	partProperties, qrCode \n" +
					") \n" +
					"VALUES (?, ?, (\n" +
					"	SELECT attachedToLeft + 1 \n" +
					"	FROM (SELECT * FROM Part) AS tbl1 \n" +
					"	WHERE partName = ? \n" +
					"), ( \n" +
					"	SELECT attachedToLeft + 2 \n" +
					"	FROM (SELECT * FROM Part) AS tbl2 \n" +
					"	WHERE partName = ? \n" +
					"), ?, ?, ?, ?, ?, ?, ?, ?);",
					partName, partType.partTypeNumber, parentPartName, parentPartName,  
					positionX, positionY, positionZ, rotationX, rotationY, rotationZ, 
					partProperties, qrCodeFileContent);
		}
		System.out.println("Done");
	}
}
