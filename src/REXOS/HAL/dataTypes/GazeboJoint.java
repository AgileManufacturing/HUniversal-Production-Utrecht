package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class GazeboJoint implements Serializable {
	private static final long serialVersionUID = 4744706739412950824L;
	
	public static final String JOINT_NAME = "jointName";
	public static final String MAX_ERROR_POSE = "maxErrorPose";
	
	private static final String getGazeboJointsForModuleType =
			"SELECT gazeboModel, jointName, maxErrorPose \n" + 
			"FROM GazeboJoint \n" + 
			"WHERE gazeboModel = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getGazeboJointsForGazeboModel =
			"SELECT gazeboModel, jointName, maxErrorPose \n" + 
			"FROM GazeboJoint \n" + 
			"WHERE gazeboModel = ?;"; 
	private static final String getGazeboJointsForPartType =
			"SELECT gazeboModel, jointName, maxErrorPose \n" + 
			"FROM GazeboJoint \n" + 
			"WHERE gazeboModel = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM PartType \n" + 
			"	WHERE typeNumber = ? \n" + 
			");";
	private static final String addGazeboJoint =
			"INSERT INTO GazeboJoint \n" + 
			"(gazeboModel, jointName, maxErrorPose) \n" + 
			"VALUES(?, ?, ?);";
	private static final String updateGazeboJoint =
			"UPDATE GazeboJoint \n" + 
			"SET maxErrorPose = ? \n" +
			"WHERE gazeboModel = ? AND \n" + 
			"	jointName = ?";
	private static final String removeGazeboJoint =
			"DELETE FROM GazeboJoint \n" + 
			"WHERE gazeboModel = ? AND \n" + 
			"	jointName = ?";
	
	public GazeboModel gazeboModel;
	public String jointName;
	public double maxErrorPose;
	
	public GazeboJoint() {
	}
	public GazeboJoint(GazeboModel gazeboModel, String jointName, double maxErrorPose) {
		this.gazeboModel = gazeboModel;
		this.jointName = jointName;
		this.maxErrorPose = maxErrorPose;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboJoint> getGazeboJointsForModuleIdentifier(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboJointsForModuleType, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		
		ArrayList<GazeboJoint> joints = new ArrayList<GazeboJoint>();
		for (Row row : rows) {
			String jointName = (String) row.get("jointName");
			double maxErrorPose = (double) row.get("maxErrorPose");
			
			GazeboModel gazeboModel = GazeboModel.getGazeboModelForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
			joints.add(new GazeboJoint(gazeboModel, jointName, maxErrorPose));
		}
		return joints;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboJoint> getGazeboJointsForGazeboModel(GazeboModel model, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboJointsForGazeboModel, model.id);
		
		ArrayList<GazeboJoint> joints = new ArrayList<GazeboJoint>();
		for (Row row : rows) {
			String jointName = (String) row.get("jointName");
			double maxErrorPose = (double) row.get("maxErrorPose");
			
			joints.add(new GazeboJoint(model, jointName, maxErrorPose));
		}
		return joints;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the part. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboJoint> getGazeboJointsForPartTypeNumber(String partTypeNumber, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboJointsForPartType, partTypeNumber);
		
		ArrayList<GazeboJoint> joints = new ArrayList<GazeboJoint>();
		for (Row row : rows) {
			String jointName = (String) row.get("jointName");
			double maxErrorPose = (double) row.get("maxErrorPose");
			
			GazeboModel gazeboModel = GazeboModel.getGazeboModelForPartTypeNumber(partTypeNumber, knowledgeDBClient);
			joints.add(new GazeboJoint(gazeboModel, jointName, maxErrorPose));
		}
		return joints;
	}
	public static GazeboJoint deSerialize(JSONObject input, GazeboModel gazeboModel) throws JSONException {
		GazeboJoint output = new GazeboJoint();
		
		output.gazeboModel = gazeboModel;
		output.jointName = input.getString(JOINT_NAME);
		output.maxErrorPose = input.getDouble(MAX_ERROR_POSE);
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(JOINT_NAME, jointName);
		output.put(MAX_ERROR_POSE, maxErrorPose);
		
		return output;
	}
	
	/**
	 * Deserializes java software from a JSONObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param javaSoftware
	 * @param knowledgeDBClient
	 * @return the now stored java software
	 * @throws JSONException 
	 */
	public void insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(addGazeboJoint, 
				gazeboModel.id, jointName, maxErrorPose);
	}
	/**
	 * This method will update the java software in the knowledge database. 
	 * @param javaSoftware
	 * @throws JSONException 
	 */
	public void updateGazeboModel(GazeboJoint gazeboJointToBeUpdated, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(updateGazeboJoint, 
				maxErrorPose,  
				gazeboJointToBeUpdated.gazeboModel.id, gazeboJointToBeUpdated.jointName);
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeGazeboJoint, gazeboModel.id, jointName);
	}
}
