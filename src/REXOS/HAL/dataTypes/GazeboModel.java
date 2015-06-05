package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

import org.apache.commons.codec.binary.Base64;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.Row;

public class GazeboModel implements Serializable {
	private static final long serialVersionUID = -223660362496024821L;
	
	public static final String BUILD_NUMBER = "buildNumber";
	public static final String ZIP_FILE = "zipFile";
	public static final String SDF_FILE_NAME = "sdfFilename";
	public static final String PARENT_LINK = "parentLink";
	public static final String CHILD_LINK = "childLink";
	public static final String CHILD_LINK_OFFSET_X = "childLinkOffsetX";
	public static final String CHILD_LINK_OFFSET_Y = "childLinkOffsetY";
	public static final String CHILD_LINK_OFFSET_Z = "childLinkOffsetZ";
	public static final String COLLISIONS = "collisions";
	public static final String JOINTS = "joints";
	public static final String LINKS = "links";
	
	private static final String getGazeboModelForModuleType =
			"SELECT id, buildNumber, sdfFilename, parentLink, childLink, zipFile, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ \n" + 
			"FROM GazeboModel \n" + 
			"WHERE id = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM ModuleType \n" + 
			"	WHERE manufacturer = ? AND \n" + 
			"		typeNumber = ? \n" + 
			");"; 
	private static final String getGazeboModelForPartType =
			"SELECT id, buildNumber, sdfFilename, parentLink, childLink, zipFile, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ \n" + 
			"FROM GazeboModel \n" + 
			"WHERE id = ( \n" + 
			"	SELECT gazeboModel \n" + 
			"	FROM PartType \n" + 
			"	WHERE typeNumber = ? \n" + 
			");"; 
	private static final String addGazeboModel =
			"INSERT INTO GazeboModel \n" + 
			"(buildNumber, sdfFilename, parentLink, childLink, zipFile, childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ) \n" + 
			"VALUES(?, ?, ?, ?, ?, ?, ?, ?);";
	private static final String updateGazeboModel =
			"UPDATE GazeboModel \n" + 
			"SET buildNumber =? AND \n" + 
			"sdfFilename = ? AND \n" + 
			"parentLink = ? AND \n" + 
			"childLink = ? AND \n" + 
			"childLinkOffsetX = ? AND \n" + 
			"childLinkOffsetY = ? AND \n" + 
			"childLinkOffsetZ = ? AND \n" + 
			"zipFile = ? \n;" + 
			"WHERE id = ?;";
	private static final String removeGazeboModel =
			"DELETE FROM GazeboModel \n" + 
			"WHERE id = ?;";
	
	/**
	 * The map used for the multiton pattern
	 */
	private static HashMap<Integer, GazeboModel> gazeboModelInstances = new HashMap<Integer, GazeboModel>();
	
	public Integer id;
	public int buildNumber;
	public byte[] zipFile;
	public String sdfFilename;
	public String parentLink;
	public String childLink;
	public double childLinkOffsetX;
	public double childLinkOffsetY;
	public double childLinkOffsetZ;
	ArrayList<GazeboCollision> collisions;
	ArrayList<GazeboJoint> joints;
	ArrayList<GazeboLink> links;
	
	public GazeboModel() {
		id = null;
		collisions = new ArrayList<GazeboCollision>();
		joints = new ArrayList<GazeboJoint>();
		links = new ArrayList<GazeboLink>();
	}
	public GazeboModel(int id, int buildNumber, byte[] zipFile, String sdfFilename, String parentLink, String childLink, 
			double childLinkOffsetX, double childLinkOffsetY, double childLinkOffsetZ, 
			ArrayList<GazeboCollision> collisions, ArrayList<GazeboJoint> joints, ArrayList<GazeboLink> links) {
		this.id = id;
		this.buildNumber = buildNumber;
		this.zipFile = zipFile;
		this.sdfFilename = sdfFilename;
		this.parentLink = parentLink;
		this.childLink = childLink;
		this.childLinkOffsetX = childLinkOffsetX;
		this.childLinkOffsetY = childLinkOffsetY;
		this.childLinkOffsetZ = childLinkOffsetZ;
		this.collisions = collisions;
		this.joints = joints;
		this.links = links;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the module. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static GazeboModel getGazeboModelForModuleIdentifier(ModuleTypeIdentifier moduleIdentifier, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboModelForModuleType, moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		
		if(rows.length != 1) {
			return null;
		}
		int id = (Integer) rows[0].get("id");
		if(gazeboModelInstances.containsKey(id) == true) {
			return gazeboModelInstances.get(id);
		}
		
		GazeboModel gazeboModel = new GazeboModel();
		gazeboModel.id = id;
		gazeboModel.buildNumber = (Integer) rows[0].get("buildNumber");
		gazeboModel.sdfFilename = (String) rows[0].get("sdfFilename");
		gazeboModel.parentLink = (String) rows[0].get("parentLink");
		gazeboModel.childLink = (String) rows[0].get("childLink");
		gazeboModel.zipFile = (byte[]) rows[0].get("zipFile");
		gazeboModel.childLinkOffsetX = (double) rows[0].get("childLinkOffsetX");
		gazeboModel.childLinkOffsetY = (double) rows[0].get("childLinkOffsetY");
		gazeboModel.childLinkOffsetZ = (double) rows[0].get("childLinkOffsetZ");
		gazeboModel.collisions = GazeboCollision.getGazeboCollisionsForGazeboModel(gazeboModel, knowledgeDBClient);
		gazeboModel.joints = GazeboJoint.getGazeboJointsForGazeboModel(gazeboModel, knowledgeDBClient);
		gazeboModel.links = GazeboLink.getGazeboLinksForGazeboModel(gazeboModel, knowledgeDBClient);
		
		return gazeboModel;
	}
	
	/**
	 * This method will get the JavaSoftware associated with the part. 
	 * @param moduleIdentifier
	 * @return
	 */
	public static GazeboModel getGazeboModelForPartTypeNumber(String partTypeNumber, KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getGazeboModelForPartType, partTypeNumber);
		
		if(rows.length != 1) {
			return null;
		}
		int id = (Integer) rows[0].get("id");
		if(gazeboModelInstances.containsKey(id) == true) {
			return gazeboModelInstances.get(id);
		}
		
		GazeboModel gazeboModel = new GazeboModel();
		gazeboModel.id = id;
		gazeboModel.buildNumber = (Integer) rows[0].get("buildNumber");
		gazeboModel.sdfFilename = (String) rows[0].get("sdfFilename");
		gazeboModel.parentLink = (String) rows[0].get("parentLink");
		gazeboModel.childLink = (String) rows[0].get("childLink");
		gazeboModel.zipFile = (byte[]) rows[0].get("zipFile");
		gazeboModel.childLinkOffsetX = (double) rows[0].get("childLinkOffsetX");
		gazeboModel.childLinkOffsetY = (double) rows[0].get("childLinkOffsetY");
		gazeboModel.childLinkOffsetZ = (double) rows[0].get("childLinkOffsetZ");
		gazeboModel.collisions = GazeboCollision.getGazeboCollisionsForGazeboModel(gazeboModel, knowledgeDBClient);
		gazeboModel.joints = GazeboJoint.getGazeboJointsForGazeboModel(gazeboModel, knowledgeDBClient);
		gazeboModel.links = GazeboLink.getGazeboLinksForGazeboModel(gazeboModel, knowledgeDBClient);
		
		return gazeboModel;
	}
	public static GazeboModel deSerialize(JSONObject input) throws JSONException {
		GazeboModel output = new GazeboModel();
		
		output.buildNumber = input.getInt(BUILD_NUMBER);
		output.zipFile = Base64.decodeBase64(input.getString(ZIP_FILE).getBytes());
		output.sdfFilename = input.getString(SDF_FILE_NAME);
		output.parentLink = input.getString(PARENT_LINK);
		output.childLink = input.getString(CHILD_LINK);
		output.childLinkOffsetX = input.getDouble(CHILD_LINK_OFFSET_X);
		output.childLinkOffsetY = input.getDouble(CHILD_LINK_OFFSET_Y);
		output.childLinkOffsetZ = input.getDouble(CHILD_LINK_OFFSET_Z);
		
		JSONArray collisions = input.getJSONArray(COLLISIONS);
		for(int i = 0; i < collisions.length(); i++) {
			output.collisions.add(GazeboCollision.deSerialize(collisions.getJSONObject(i), output));
		}
		JSONArray joints = input.getJSONArray(JOINTS);
		for(int i = 0; i < joints.length(); i++) {
			output.joints.add(GazeboJoint.deSerialize(joints.getJSONObject(i), output));
		}
		JSONArray links = input.getJSONArray(LINKS);
		for(int i = 0; i < links.length(); i++) {
			output.links.add(GazeboLink.deSerialize(links.getJSONObject(i), output));
		}
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(BUILD_NUMBER, buildNumber);
		output.put(ZIP_FILE, new String(Base64.encodeBase64(zipFile)));
		output.put(SDF_FILE_NAME, sdfFilename);
		output.put(PARENT_LINK, parentLink);
		output.put(CHILD_LINK, childLink);
		output.put(CHILD_LINK_OFFSET_X, childLinkOffsetX);
		output.put(CHILD_LINK_OFFSET_Y, childLinkOffsetY);
		output.put(CHILD_LINK_OFFSET_Z, childLinkOffsetZ);
		
		JSONArray collisionsJson = new JSONArray();
		for (GazeboCollision collision : collisions) {
			collisionsJson.put(collision.serialize());
		}
		output.put(COLLISIONS, collisionsJson);
		
		JSONArray jointsJson = new JSONArray();
		for (GazeboJoint joint : joints) {
			jointsJson.put(joint.serialize());
		}
		output.put(JOINTS, jointsJson);
		
		JSONArray linksJson = new JSONArray();
		for (GazeboLink link : links) {
			linksJson.put(link.serialize());
		}
		output.put(LINKS, linksJson);
		
		return output;
	}
	
	/**
	 * Deserializes java software from a JSONObject and stores in in the knowledge database using the provided KnowledgeDBClient.
	 * @param javaSoftware
	 * @param knowledgeDBClient
	 * @return the now stored java software
	 * @throws JSONException 
	 */
	public int insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) {
		int id = knowledgeDBClient.executeUpdateQuery(addGazeboModel, 
				buildNumber, sdfFilename, parentLink, childLink, zipFile, 
				childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ);
		this.id = id;
		
		for (GazeboCollision collision : collisions) {
			collision.insertIntoDatabase(knowledgeDBClient);
		}
		for (GazeboJoint joint : joints) {
			joint.insertIntoDatabase(knowledgeDBClient);
		}
		for (GazeboLink link : links) {
			link.insertIntoDatabase(knowledgeDBClient);
		}
		
		gazeboModelInstances.put(id, this);
		return id;
	}
	/**
	 * This method will update the java software in the knowledge database. 
	 * @param javaSoftware
	 * @throws JSONException 
	 */
	public void updateGazeboModel(GazeboModel gazeboModelToBeUpdated, KnowledgeDBClient knowledgeDBClient) {
		knowledgeDBClient.executeUpdateQuery(updateGazeboModel, 
				buildNumber, sdfFilename, parentLink, childLink, zipFile, 
				childLinkOffsetX, childLinkOffsetY, childLinkOffsetZ, 
				gazeboModelToBeUpdated.id);
		for (GazeboCollision collision : gazeboModelToBeUpdated.collisions) {
			collision.removeFromDatabase(knowledgeDBClient);
		}
		for (GazeboCollision collision : collisions) {
			collision.insertIntoDatabase(knowledgeDBClient);
		}
		
		for (GazeboJoint joint : gazeboModelToBeUpdated.joints) {
			joint.removeFromDatabase(knowledgeDBClient);
		}
		for (GazeboJoint joint : joints) {
			joint.insertIntoDatabase(knowledgeDBClient);
		}
		
		for (GazeboLink link : gazeboModelToBeUpdated.links) {
			link.removeFromDatabase(knowledgeDBClient);
		}
		for (GazeboLink link : links) {
			link.insertIntoDatabase(knowledgeDBClient);
		}
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		for (GazeboCollision collision : collisions) {
			collision.removeFromDatabase(knowledgeDBClient);
		}
		for (GazeboJoint joint : joints) {
			joint.removeFromDatabase(knowledgeDBClient);
		}
		for (GazeboLink link : links) {
			link.removeFromDatabase(knowledgeDBClient);
		}
		knowledgeDBClient.executeUpdateQuery(removeGazeboModel, id);
	}
}
