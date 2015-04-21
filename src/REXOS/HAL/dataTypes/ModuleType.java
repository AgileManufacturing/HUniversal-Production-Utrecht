package HAL.dataTypes;

import java.io.Serializable;
import java.util.ArrayList;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.omg.PortableInterceptor.INACTIVE;

import util.log.LogLevel;
import util.log.LogSection;
import util.log.Logger;
import HAL.libraries.knowledgedb_client.KnowledgeDBClient;
import HAL.libraries.knowledgedb_client.KnowledgeException;
import HAL.libraries.knowledgedb_client.Row;

public class ModuleType implements Serializable {
	private static final long serialVersionUID = 4432359335483148571L;
	
	public static final String PROPERTIES = "properties";
	public static final String ROS_SOFTWARE = "rosSoftware";
	public static final String HAL_SOFTWARE = "halSoftware";
	public static final String GAZEBO_MODEL = "gazeboModel";
	public static final String SUPPORTED_MUTATIONS = "supportedMutations";
	public static final String CAPABILITIES = "capabilities";
	
	/**
	 * SQL query for selecting all the data of moduleType.
	 * Input: moduleTypeManufacturer, moduleTypeTypeNumber
	 */
	private static final String getModuleType = 
			"SELECT * \n" +
			"FROM ModuleType \n" +
			"WHERE manufacturer = ? AND \n" +
			"	typeNumber = ?;";
	/**
	 * SQL query for removing all the moduleType which are obsolete.
	 * Input: -
	 * A moduleType is considered to be obsolete if there are no modules of that type connected to any equiplet.
	 */
	private static final String getModuleTypesWithNoModules = 
			"SELECT manufacturer, typeNumber \n" +
			"FROM ModuleType \n" +
			"WHERE NOT EXISTS( \n" +
			"	SELECT * \n" +
			"	FROM Module \n" +
			"	WHERE manufacturer = ModuleType.manufacturer AND \n" +
			"		typeNumber = ModuleType.typeNumber \n" +
			");";
	private static final String getPropertiesForModuleIdentifier = 
			"SELECT moduleTypeProperties \n" +
			"FROM ModuleType \n" +
			"WHERE manufacturer = ? AND \n" +
			"typeNumber = ?;";
	/**
	 * SQL query for adding a moduleType. Input: moduleTypeManufacturer,
	 * moduleTypeTypeNumber, halSoftwareId, rosSoftwareId Ignored if a record
	 * with the same primary key already exists.
	 */
	private static final String addModuleType = 
			"INSERT IGNORE INTO ModuleType \n" +
			"(manufacturer, typeNumber, moduleTypeProperties, halSoftware, rosSoftware, gazeboModel) \n" +
			"VALUES (?, ?, ?, ?, ?, ?);";
	/**
	 * SQL query for removing a module. Input: moduleManufacturer,
	 * moduleTypeNumber, moduleSerialNumber
	 */
	private static final String removeModuleType = 
			"DELETE FROM ModuleType \n" +
			"WHERE manufacturer = ? AND \n" +
			"	typeNumber = ?;";
	
	public ModuleTypeIdentifier moduleTypeIdentifier;
	public JSONObject properties;
	public RosSoftware rosSoftware;
	public JavaSoftware halSoftware;
	public GazeboModel gazeboModel;
	public ArrayList<Mutation> supportedMutations;
	public ArrayList<SerializedCapability> capabilities;
	
	public ModuleType() {
		supportedMutations = new ArrayList<Mutation>();
		capabilities = new ArrayList<SerializedCapability>();
	}
	
	public static ModuleType getSerializedModuleTypeByModuleTypeIdentifier(ModuleTypeIdentifier moduleIdentifier, 
			KnowledgeDBClient knowledgeDBClient) throws JSONException {
		ModuleType output = new ModuleType();
		output.moduleTypeIdentifier = moduleIdentifier;
		
		Row[] rows = knowledgeDBClient.executeSelectQuery(getPropertiesForModuleIdentifier, 
				moduleIdentifier.manufacturer, moduleIdentifier.typeNumber);
		JSONTokener tokener = new JSONTokener((String) rows[0].get("moduleTypeProperties"));
		output.properties = new JSONObject(tokener);
		
		output.halSoftware = JavaSoftware.getJavaSoftwareForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
		// TODO what if null?
		output.rosSoftware = RosSoftware.getRosSoftwareForModuleIdentifier(moduleIdentifier, knowledgeDBClient);
		output.gazeboModel = GazeboModel.getGazeboModelForModuleIdentifier(moduleIdentifier, knowledgeDBClient);

		output.supportedMutations = Mutation.getSupportedMutations(moduleIdentifier, knowledgeDBClient);
		
		output.capabilities = SerializedCapability.getCapabilitiesFromDatabase(moduleIdentifier, knowledgeDBClient);
		
		return output;
	}
	
	public static ModuleType deSerialize(JSONObject input, ModuleIdentifier moduleTypeIdentifier) throws JSONException {
		ModuleType output = new ModuleType();
		
		output.moduleTypeIdentifier = moduleTypeIdentifier;
		output.properties = input.getJSONObject(PROPERTIES);
		if(input.isNull(ROS_SOFTWARE) == false) {
			output.rosSoftware = RosSoftware.deSerialize(input.getJSONObject(ROS_SOFTWARE));
		}
		output.halSoftware = JavaSoftware.deSerialize(input.getJSONObject(HAL_SOFTWARE));
		output.gazeboModel = GazeboModel.deSerialize(input.getJSONObject(GAZEBO_MODEL));
		
		JSONArray supportedMutations = input.getJSONArray(SUPPORTED_MUTATIONS);
		for (int i = 0; i < supportedMutations.length(); i++) {
			output.supportedMutations.add(Mutation.deSerialize(supportedMutations.getString(i)));
		}
		
		JSONArray capabilities = input.getJSONArray(CAPABILITIES);
		for (int i = 0; i < capabilities.length(); i++) {
			output.capabilities.add(SerializedCapability.deSerialize(capabilities.getJSONObject(i)));
		}
		
		return output;
	}
	public JSONObject serialize() throws JSONException {
		JSONObject output = new JSONObject();
		
		output.put(PROPERTIES, properties);
		output.put(ROS_SOFTWARE, rosSoftware.serialize());
		output.put(HAL_SOFTWARE, halSoftware.serialize());
		output.put(GAZEBO_MODEL, gazeboModel.serialize());
		
		JSONArray supportedMutations = new JSONArray();
		for (int i = 0; i < this.supportedMutations.size(); i++) {
			supportedMutations.put(this.supportedMutations.get(i).serialize());
		}
		output.put(SUPPORTED_MUTATIONS, supportedMutations);
		
		JSONArray capabilities = new JSONArray();
		for (int i = 0; i < this.capabilities.size(); i++) {
			capabilities.put(this.capabilities.get(i).serialize());
		}
		output.put(CAPABILITIES, capabilities);
		
		return output;
	}
	
	/**
	 * This method inserts a moduleType in the knowledge database.
	 * 
	 * @param moduleIdentifier
	 * @param type
	 * @return
	 * @throws KnowledgeException
	 * @throws JSONException
	 */
	public void insertIntoDatabase(KnowledgeDBClient knowledgeDBClient) throws JSONException {
		if(existsInDatabase(knowledgeDBClient) == true) {
			updateModuleType(knowledgeDBClient);
		} else {
			int halSoftwareId = halSoftware.insertIntoDatabase(knowledgeDBClient);
			
			// not every module has rosSoftware
			Integer rosSoftwareId = null;
			if(rosSoftware != null) {
				rosSoftwareId = rosSoftware.insertIntoDatabase(knowledgeDBClient);
			}
			
			int gazeboModelId = gazeboModel.insertIntoDatabase(knowledgeDBClient);
			
			knowledgeDBClient.executeUpdateQuery(addModuleType, 
					moduleTypeIdentifier.manufacturer, moduleTypeIdentifier.typeNumber, 
					properties, halSoftwareId, rosSoftwareId, gazeboModelId);
			
			for (SerializedCapability capability : capabilities) {
				capability.insertIntoDatabase(knowledgeDBClient);
			}
			
			Mutation.insertSupportedMutations(moduleTypeIdentifier, supportedMutations, knowledgeDBClient); 
		}
	}
	/**
	 * This method updates a moduleType in the knowledge database. It will
	 * update the software of the moduleType if the buildNumber of the provided
	 * software is higher than the buildNumber of the currently stored software.
	 * 
	 * @param moduleIdentifier
	 * @param type
	 * @throws JSONException
	 */
	private void updateModuleType(KnowledgeDBClient knowledgeDBClient) {
		if(halSoftware != null) {
			JavaSoftware currentJavaSoftware = JavaSoftware.getJavaSoftwareForModuleIdentifier(moduleTypeIdentifier, knowledgeDBClient);
			int currentJavaBuildNumber;
			if(currentJavaSoftware == null) currentJavaBuildNumber = Integer.MIN_VALUE;
			else currentJavaBuildNumber = currentJavaSoftware.buildNumber;
			
			int newJavaBuildNumber = halSoftware.buildNumber;
			if (newJavaBuildNumber > currentJavaBuildNumber) {
				// update the halSoftware
				Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.INFORMATION, "Updating HAL software for module " + moduleTypeIdentifier);
				halSoftware.updateDatabase(currentJavaSoftware, knowledgeDBClient);
			}
		}
		
		if(rosSoftware != null) {
			RosSoftware currentRosSoftware = RosSoftware.getRosSoftwareForModuleIdentifier(moduleTypeIdentifier, knowledgeDBClient);
			int currentRosBuildNumber;
			if(currentRosSoftware == null) currentRosBuildNumber = Integer.MIN_VALUE;
			else currentRosBuildNumber = currentRosSoftware.buildNumber;
			
			int newRosBuildNumber = rosSoftware.buildNumber;
			if (newRosBuildNumber > currentRosBuildNumber) {
				// update the rosSoftware
				Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.INFORMATION, "Updating ROS software for module " + moduleTypeIdentifier);
				rosSoftware.updateDatabase(currentRosSoftware, knowledgeDBClient);
			}
		}
		
		if(gazeboModel != null) {	
			GazeboModel currentGazeboModel = GazeboModel.getGazeboModelForModuleIdentifier(moduleTypeIdentifier, knowledgeDBClient);
			int currentModelBuildNumber;
			if(currentGazeboModel == null) currentModelBuildNumber = Integer.MIN_VALUE;
			else currentModelBuildNumber = currentGazeboModel.buildNumber;
			
			int newModelBuildNumber = gazeboModel.buildNumber;
			if (newModelBuildNumber > currentModelBuildNumber) {
				// update the gazeboModel
				Logger.log(LogSection.HAL_MODULE_FACTORY, LogLevel.INFORMATION, "Updating Gazebo model for module " + moduleTypeIdentifier);
				gazeboModel.updateGazeboModel(currentGazeboModel, knowledgeDBClient);
			}
		}
		
		for (SerializedCapability capability : capabilities) {
			capability.insertIntoDatabase(knowledgeDBClient);
		}
	}
	public void removeFromDatabase(KnowledgeDBClient knowledgeDBClient) {
		Mutation.removeSupportedMutations(moduleTypeIdentifier, knowledgeDBClient);
		SerializedCapability.removeAllUnusedCapabilitiesFromDatabase(knowledgeDBClient);
		
		knowledgeDBClient.executeUpdateQuery(removeModuleType, moduleTypeIdentifier.manufacturer, moduleTypeIdentifier.typeNumber);
		
		halSoftware.removeFromDatabase(knowledgeDBClient);
		rosSoftware.removeFromDatabase(knowledgeDBClient);
		gazeboModel.removeFromDatabase(knowledgeDBClient);
	}
	public static void removeUnusedFromDatabase(KnowledgeDBClient knowledgeDBClient) throws JSONException {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleTypesWithNoModules);
		for (Row row : rows) {
			ModuleTypeIdentifier moduleIdentifier = new ModuleTypeIdentifier((String) row.get("manufacturer"), (String) row.get("typeNumber"));
			ModuleType module = ModuleType.getSerializedModuleTypeByModuleTypeIdentifier(moduleIdentifier, knowledgeDBClient);
			module.removeFromDatabase(knowledgeDBClient);
		}
	}
	/**
	 * This method determines if a moduleType (identified by the {@link ModuleIdentifier}) is known in the knowledge database.
	 * 
	 * @param moduleIdentifier
	 * @return true if the moduleType is known in the knowledge database, false otherwise.
	 */
	public boolean existsInDatabase(KnowledgeDBClient knowledgeDBClient) {
		Row[] rows = knowledgeDBClient.executeSelectQuery(getModuleType, moduleTypeIdentifier.manufacturer, moduleTypeIdentifier.typeNumber);
		if (rows.length == 1) {
			return true;
		} else {
			return false;
		}
	}

}
