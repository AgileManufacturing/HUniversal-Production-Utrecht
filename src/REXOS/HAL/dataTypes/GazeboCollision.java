package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;

import org.json.JSONException;
import org.json.JSONObject;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class GazeboCollision implements Serializable {
	private static final long serialVersionUID = 4420116520764366301L;
	
	public static final String LINK_NAME = "linkName";
	public static final String COLLISION_NAME = "collisionName";
	public static final String MAX_FORCE = "maxForce";
	public static final String MAX_TORQUE = "maxTorque";
	public static final String MAY_HAVE_CONTACT_WITH_CHILD_MODULES = "mayHaveContactWithChildModules";
	
	private static final String getGazeboCollisionsForModuleType =
			"SELECT gazeboModel, linkName, collisionName, maxForce, maxTorque, mayHaveContactWithChildModules \n" + 
			"FROM GazeboCollision \n" + 
			"WHERE gazeboModel = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getGazeboCollisionsForGazeboModel =
			"SELECT gazeboModel, linkName, collisionName, maxForce, maxTorque, mayHaveContactWithChildModules \n" + 
			"FROM GazeboCollision \n" + 
			"WHERE gazeboModel = ?;"; 
	private static final String getGazeboCollisionsForPartType =
			"SELECT gazeboModel, linkName, collisionName, maxForce, maxTorque, mayHaveContactWithChildModules \n" + 
			"FROM GazeboCollision \n" + 
			"WHERE gazeboModel = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM PartType \n" + 
			"	WHERE typeNumber = ? \n" + 
			");";
	private static final String addGazeboCollision =
			"INSERT INTO GazeboCollision \n" + 
			"(gazeboModel, linkName, collisionName, maxForce, maxTorque, mayHaveContactWithChildModules) \n" + 
			"VALUES(?, ?, ?, ?, ?, ?);";
	private static final String updateGazeboCollision =
			"UPDATE GazeboCollision \n" + 
			"SET maxForce = ? AND \n" +
			"maxTorque = ? AND \n" +
			"mayHaveContactWithChildModules = ? \n" +
			"WHERE gazeboModel = ? AND \n" + 
			"	linkName = ? AND \n" + 
			"	collisionName = ?";
	private static final String removeGazeboCollision =
			"DELETE FROM GazeboCollision \n" + 
			"WHERE gazeboModel = ? AND \n" + 
			"	linkName = ? AND \n" + 
			"	collisionName = ?";
	
	public GazeboModel gazeboModel;
	public String linkName;
	public String collisionName;
	public double maxForce;
	public double maxTorque;
	public boolean mayHaveContactWithChildModules;
	
	public GazeboCollision() {
	}
	public GazeboCollision(GazeboModel gazeboModel, String linkName, String collisionName, 
			double maxForce, double maxTorque, boolean mayHaveContactWithChildModules) {
		this.gazeboModel = gazeboModel;
		this.linkName = linkName;
		this.collisionName = collisionName;
		this.maxForce = maxForce;
		this.maxTorque = maxTorque;
		this.mayHaveContactWithChildModules = mayHaveContactWithChildModules;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboCollision> getGazeboCollisionsForModuleIdentifier(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboCollisionsForModuleType, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		
		ArrayList<GazeboCollision> collisions = new ArrayList<GazeboCollision>();
		for (Row row : rows) {
			String linkName = (String) row.get("linkName");
			String collisionName = (String) row.get("collisionName");
			double maxForce = (double) row.get("maxForce");
			double maxTorque = (double) row.get("maxTorque");
			boolean mayHaveContactWithChildModules = (boolean) row.get("mayHaveContactWithChildModules");
			
			GazeboModel gazeboModel = GazeboModel.getGazeboModelForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
			collisions.add(new GazeboCollision(gazeboModel, linkName, collisionName, 
					maxForce, maxTorque, mayHaveContactWithChildModules));
		}
		return collisions;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboCollision> getGazeboCollisionsForGazeboModel(GazeboModel model, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboCollisionsForGazeboModel, model.id);
		
		ArrayList<GazeboCollision> collisions = new ArrayList<GazeboCollision>();
		for (Row row : rows) {
			String linkName = (String) row.get("linkName");
			String collisionName = (String) row.get("collisionName");
			double maxForce = (double) row.get("maxForce");
			double maxTorque = (double) row.get("maxTorque");
			boolean mayHaveContactWithChildModules = (boolean) row.get("mayHaveContactWithChildModules");
			
			collisions.add(new GazeboCollision(model, linkName, collisionName, 
					maxForce, maxTorque, mayHaveContactWithChildModules));
		}
		return collisions;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the part. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static ArrayList<GazeboCollision> getGazeboCollisionsForPartTypeNumber(String partTypeNumber, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboCollisionsForPartType, partTypeNumber);
		
		ArrayList<GazeboCollision> collisions = new ArrayList<GazeboCollision>();
		for (Row row : rows) {
			String linkName = (String) row.get("linkName");
			String collisionName = (String) row.get("collisionName");
			double maxForce = (double) row.get("maxForce");
			double maxTorque = (double) row.get("maxTorque");
			boolean mayHaveContactWithChildModules = (boolean) row.get("mayHaveContactWithChildModules");
			
			GazeboModel gazeboModel = GazeboModel.getGazeboModelForPartTypeNumber(partTypeNumber, knowledgeDBClient);
			collisions.add(new GazeboCollision(gazeboModel, linkName, collisionName, 
					maxForce, maxTorque, mayHaveContactWithChildModules));
		}
		return collisions;
	}
	public static GazeboCollision deSerialize(JSONObject input, GazeboModel gazeboModel) throws JSONException {
		GazeboCollision output = new GazeboCollision();
		
		output.gazeboModel = gazeboModel;
		output.linkName = input.getString(LINK_NAME);
		output.collisionName = input.getString(COLLISION_NAME);
		output.maxForce = input.getDouble(MAX_FORCE);
		output.maxTorque = input.getDouble(MAX_TORQUE);
		output.mayHaveContactWithChildModules = input.getBoolean(MAY_HAVE_CONTACT_WITH_CHILD_MODULES);
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(LINK_NAME, linkName);
		output.put(COLLISION_NAME, collisionName);
		output.put(MAX_FORCE, maxForce);
		output.put(MAX_TORQUE, maxTorque);
		output.put(MAY_HAVE_CONTACT_WITH_CHILD_MODULES, mayHaveContactWithChildModules);
		
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
		knowledgeDBClient.executeUpdateQuery(addGazeboCollision, 
				gazeboModel.id, linkName, collisionName, maxForce, maxTorque, mayHaveContactWithChildModules);
	}
	/**
	 * This method will update the java software in the knowledge database. 
	 * @param javaSoftware
	 * @throws JSONException 
	 */
	public void updateGazeboModel(GazeboCollision gazeboCollisionToBeUpdated, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(updateGazeboCollision, 
				maxForce, maxTorque, mayHaveContactWithChildModules, 
				gazeboCollisionToBeUpdated.gazeboModel.id, gazeboCollisionToBeUpdated.linkName, gazeboCollisionToBeUpdated.collisionName);
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(removeGazeboCollision, gazeboModel.id, linkName, collisionName);
	}
}
